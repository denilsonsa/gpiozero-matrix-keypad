#!/usr/bin/env python3

print('''
##### WARNING #####
This code is FAR from ready.


I tested this code with a 3x4 keypad (`12345679*0#`). It doesn't work well.
The probing code seems to work, but everything else around it is messy and
unreliable. I'm abandoning this code for now.

I originally tried to pick the best of these two pieces of code:
https://github.com/brettmclean/pad4pi/blob/develop/pad4pi/rpi_gpio.py
https://github.com/adafruit/Adafruit_CircuitPython_MatrixKeypad/blob/main/adafruit_matrixkeypad.py
Plus bits from gpiozero's ButtonBoard and Rotary Encoder.

My idea was to use interrupts to get notified of button state changes, and then
react to those by probing all the keys to find the real states of the buttons,
and then revert back to waiting.

Sidenote: these aren't real interrupts; the `raspberry-gpio-python` internally
runs a polling loop to look for changes, and then calls your callback. It gives
the illusion of interrupts, but they are technically not interrupts. Or maybe
they are, and I completely misunderstood the code while taking a quick look.
https://sourceforge.net/p/raspberry-gpio-python/code/ci/default/tree/source/event_gpio.c#l357

Unfortunately, it didn't work as intended. Since the polling and the callback
run in a different thread, I'm now dealing with racing conditions and all the
trouble that comes with multi-thread coding (with the extra difficulty that I
don't have control over the other threads).

When I was pressing buttons from the first two columns (`147`, `258`),
everything worked fine (or good enough). However, randomly when pressing
buttons on the third column (`369`) the code would enter an infinite callback
loop. It seemed like the `_probe_keypad` would finish the setup and reactivate
the callbacks too quickly, so the library code was detecting some edge change
on the pin 13 and calling my callback, which would trigger another probe, which
would self-sustain this madness. Why only on pin 13? Was it because it was the
last one in the columns loop. Or maybe it was luck (AKA racing condition).

My last hope was to try adding a basic debouncing code by using a `Timer`
object. Well, that works, the infinite calling of callbacks was halted. But it
also means the code is now eating inputs, so it is unreliable in a different
way.

For now, I give up on this approach. If someone else wants to give it a try, be
my guest. Maybe someone somewhere will be able to make it work.

My next approach will be simplified: the class will work in two ways: either
manual probing (the user of the class will call a method or read from a
property/generator to trigger the probing), or automatic probing (a background
thread that will probe the values at a certain frequency, with optional
debouncing).

''')

from functools import partial
from threading import Lock, Timer
import traceback

from gpiozero.devices import CompositeDevice, GPIODevice
from gpiozero.input_devices import InputDevice
from gpiozero.output_devices import OutputDevice
from gpiozero.mixins import EventsMixin, HoldMixin

class MatrixKeypad(CompositeDevice):
    # rows = list of pins (usually 4 pins)
    # cols = list of pins (usually 3 or 4 pins)
    # labels = One of:   list of list  |  list of strings
    #          In fact, it must be a sequence of sequences.
    #          Making it a tuple is good because then it's immutable. But it doesn't really matter.
    #          The dimensions must match rows x cols.
    #          Examples:
    #            ["123A", "456B", "789C", "*0#D"]
    #            [ [1, 2, 3], [4, 5, 6], [7, 8, 9]]
    # bounce_time = must be implemented by myself, because of the polling behavior across the matrix
    def __init__(self, rows, cols, labels, *,
                 # bounce_time=None, hold_time=1, hold_repeat=False,
                 pin_factory=None):
        if len(labels) == 0:
            raise ValueError("labels must not be empty")
        if len(labels) != len(rows):
            raise ValueError("labels must have as many elements as rows (pins)")
        if any(len(labelrow) != len(cols) for labelrow in labels):
            raise ValueError("Each element from labels must have the same length as cols (pins)")

        self.labels = labels
        self.row_pins = [ GPIODevice(pin, pin_factory=pin_factory) for pin in rows ]
        self.col_pins = [ GPIODevice(pin, pin_factory=pin_factory) for pin in cols ]
        super().__init__(*self.row_pins, *self.col_pins, pin_factory=pin_factory)
        self._handlers = []
        self._when_changed_lock = Lock()
        self._disable_when_changed_handler = False
        self._reset_pins()
        self._setup_pins_for_waiting()

    def _reset_pins(self):
        """
        Resets the pins to be all INPUT, avoiding any short-circuit risk.

        The rest of this class assumes this function was called at least once.
        """
        self._handlers = []
        for p in self.row_pins + self.col_pins:
            p.pin.when_changed = None

        for p in self.row_pins + self.col_pins:
            p.pin.input_with_pull('up')
            p.pin.bounce = None

    def _setup_pins_for_waiting(self):
        """
        Configures the pins so that any state change (button press or release)
        can be detected immediately. The actual state change has to be queried
        later.

        All rows are set to OUTPUT LOW, and all columns are set to INPUT with
        edge detection.
        """

        self._handlers = []
        for p in self.col_pins:
            p.pin.when_changed = None

        for p in self.row_pins:
            p.pin.output_with_state(0)

        for p in self.col_pins:
            p.pin.input_with_pull('up')
            #p.pin.when_changed = self._when_changed_handler

            f = partial(self._when_changed_handler, pinno=p.pin._number)
            self._handlers.append(f)
            p.pin.when_changed = f

            # Note: although the documentation claims "none" is a valid value,
            # it's not supported by rpigpio.py
            p.pin.edges = 'both'

    def _when_changed_handler(self, ticks=None, state=None, *, pinno=None):
        with self._when_changed_lock:
            #traceback.print_stack()
            print("called for pin {}".format(pinno))
            if self._disable_when_changed_handler:
                print('No-op')
                return
            print('Something detected ticks={} state={}'.format(ticks, state))
            self.probe()

    def _probe_keypad(self):
        """
        Reads the actual state of the keypad.

        It doesn't matter if it queries rows or columns, as the behavior and
        the number of steps would be the same. This implementation just picked
        one of two approaches.
        """

        self._disable_when_changed_handler = True
        self._reset_pins()

        pressed = set()
        for rowno, row in enumerate(self.row_pins):
            # Set this row as active.
            row.pin.output_with_state(0)

            # If a button is pressed in this row, we expect to read it from the column.
            for colno, col in enumerate(self.col_pins):
                if col.pin.state == 0:
                    print('Found row {} and col {}.'.format(rowno, colno))
                    pressed.add((rowno, colno))

            for rowno2, row2 in enumerate(self.row_pins):
                if rowno2 != rowno:
                    if row2.pin.state == 0:
                        print('Rows {} and {} are active.'.format(rowno, rowno2))

            row.pin.input_with_pull('up')

        self._disable_when_changed_handler = False
        # This timer avoids the infinite loop of calling the function.
        # But it also misses short keypresses. Dang!
        Timer(0.01, self._setup_pins_for_waiting).start()
        return pressed

    def probe(self):
        """
        Quick and dirty debugging function. Should be replaced by the proper thing (whatever that thing turns out to be).
        """

        pressed = self._probe_keypad()
        print("Pressed: " + " ".join(self.labels[rowno][colno] for (rowno, colno) in sorted(pressed)))

        # Ideas:
        # * Get inspired by https://github.com/adafruit/Adafruit_CircuitPython_MatrixKeypad/blob/main/adafruit_matrixkeypad.py
        # * Get inspired by pad4pi code, and use interrupts to detect keypresses as quickly as possible.
        # * Setup:
        #   - All pins are input.
        #   - Choose one line (either row or col), set as output. This will provide "power" to detect an edge later.
        #      - Actually, we need to set all lines.
        #   - Set all pins from the perpendicular line as detecting an edge, and provide a callback function.
        #
        # * Callback function:
        #   - Start by disabling further calls to this function until later.
        #   - Run the polling algorithm (disable all but one line, read, repeat for the next line, repeat…)
        #      - "Disable" means switching to INPUT.
        #   - The result will be zero, one, or many keys detected.
        #   - Re-enable the function. Must test if it's not being called multiple times. Looking at the ticks parameter might help.
        #
        # * We probably need multiple callback functions, one per line.
        #
        # * We should have some methods: _setup_pins_for_waiting, _probe_keys


        # Bad ideas:
        # * Check which is longer: rows or cols. Then use the longer as input,
        #   and the shorter as output. Actually, scrap that, there is no time
        #   difference.

        # TODO: Implement bounce_time ourselves. Look at line 362 from
        #       https://sourceforge.net/p/raspberry-gpio-python/code/ci/default/tree/source/event_gpio.c
        # TODO: Implement hold and hold_repeat. Look at HoldMixin.

        #raise NotImplementedError('This code is in a very early state.')

if __name__ == "__main__":
    kp = MatrixKeypad(
        # pin 21 is not used in this 3x4 matrix
        rows=[19, 26, 16, 20],
        cols=[5, 6, 13],
        labels=["123", "456", "789", "*0#"],
    )
    input('Press Enter to quit.')

# from pad4pi.rpi_gpio import KeypadFactory
# 
# def printkey(key):
#     print(key)
# 
# kp = KeypadFactory().create_keypad(
#     keypad=["123A", "456B", "789C", "*0#D"],
#     # pin 21 is not used in this 3x4 matrix
#     row_pins=[19, 26, 16, 20],
#     col_pins=[5, 6, 13],
# )
# kp.registerKeyPressHandler(printkey)
# input('Press Enter to quit.')
# kp.cleanup()
