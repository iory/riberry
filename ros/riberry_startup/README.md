# riberry startup

## atoms3_button_state_publisher.launch

The atom s3 display features a multi-functional button capable of detecting various press patterns, which are essential for implementing diverse interactive features. When the `display_information.py` is executed with the systemd service:

It publishes the state of the button on the `/atom_s3_button_state` topic as an `std_msgs/Int32` value.
Below is a table outlining these states, their descriptions, and corresponding numeric values:

| Button State      | Description                                             | Numeric Value |
|-------------------|---------------------------------------------------------|---------------|
| `NOT_CHANGED`     | No change detected in the button state since last check.| 0             |
| `SINGLE_CLICK`    | The button was clicked once.                            | 1             |
| `DOUBLE_CLICK`    | The button was clicked twice in quick succession.       | 2             |
| `TRIPLE_CLICK`    | The button was clicked three times in quick succession. | 3             |
| `QUADRUPLE_CLICK` | The button was clicked four times in quick succession.  | 4             |
| `QUINTUPLE_CLICK` | The button was clicked five times in quick succession.  | 5             |
| `SEXTUPLE_CLICK`  | The button was clicked six times in quick succession.   | 6             |
| `SEPTUPLE_CLICK`  | The button was clicked seven times in quick succession. | 7             |
| `OCTUPLE_CLICK`   | The button was clicked eight times in quick succession. | 8             |
| `NONUPLE_CLICK`   | The button was clicked nine times in quick succession.  | 9             |
| `DECUPLE_CLICK`   | The button was clicked ten times in quick succession.   | 10            |
| `PRESSED`         | The button is currently being pressed.                  | 11            |
| `RELEASED`        | The button has been released after a press.             | 12            |
