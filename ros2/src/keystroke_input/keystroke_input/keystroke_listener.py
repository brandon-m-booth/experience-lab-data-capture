#!/usr/bin/env python3

from pynput.keyboard import Listener

def on_key_press(key):
    try:
        key_char = key.char
    except AttributeError:
        key_char = str(key)

    print(f'Key pressed: {key_char}')

# Start listening for keystrokes
with Listener(on_press=on_key_press) as listener:
    listener.join()
