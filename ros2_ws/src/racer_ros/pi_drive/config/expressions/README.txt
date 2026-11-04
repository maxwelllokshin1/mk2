blinking/   expressions that blink every 10 s (with normal's blink.gif): normal, excited, sarcastic
static/     expressions that never blink: surprised, angry, dizzy, sleepy, bored

Each expression folder has eyes/ (pictures) and voice/ (.mp3 lines). See the README.txt inside each one.
Copy new files onto the Pi, then restart the car (scripts/pi/car.sh restart).

When each one shows up:
  parked      normal  surprised  angry  sarcastic  excited  sleepy  bored      (random, random times)
  driving     normal  angry  sarcastic                                         (random, random times)
  surprised   also: about to hit a wall           dizzy   ONLY: collision
  excited     also: lap completed, stopping after a long drive, the Joy-Con excite button
