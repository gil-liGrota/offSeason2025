import keyboard  # pip install keyboard
import vgamepad as vg
import time

# Create virtual Xbox 360 controller
gamepad = vg.VX360Gamepad()

print("Keyboard → Xbox controller mapping active!")
print("ESC to quit.")

while True:
    # --- Face Buttons ---
    if keyboard.is_pressed("1"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)

    if keyboard.is_pressed("2"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_X)

    if keyboard.is_pressed("3"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_B)

    if keyboard.is_pressed("4"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_Y)

    # --- D-Pad ---
    if keyboard.is_pressed("shift"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_UP)

    if keyboard.is_pressed("ctrl"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_DOWN)

    if keyboard.is_pressed("left"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_LEFT)

    if keyboard.is_pressed("right"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_DPAD_RIGHT)

    # --- Start & Back ---
    if keyboard.is_pressed("backspace"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_START)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_START)

    if keyboard.is_pressed("\\"):  # backslash
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_BACK)

    # --- Triggers ---
    if keyboard.is_pressed("u"):
        gamepad.left_trigger_float(value_float=0.9)
    else:
        gamepad.left_trigger_float(value_float=0.0)

    if keyboard.is_pressed("j"):
        gamepad.right_trigger_float(value_float=0.9)
    else:
        gamepad.right_trigger_float(value_float=0.0)

    # --- Bumpers ---
    if keyboard.is_pressed("b"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_LEFT_SHOULDER)

    if keyboard.is_pressed("m"):
        gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)
    else:
        gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_RIGHT_SHOULDER)

    # --- Update controller state ---
    gamepad.update()

    # Quit on ESC
    if keyboard.is_pressed("esc"):
        print("Exiting...")
        break

    time.sleep(0.01)
