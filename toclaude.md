# Don't care about this.
## Just for my draft for claude prompt.

1. We are useing TT-motor, so don't need lego motor things, change everything to TT-motor, remember to know the scope.
2. I want different set speed for different things, like fast, normal, slow. And everything preset normal, if I need to do speed, I can let a peticualr moving sequence fast. Slow the same way. This scope is to every chassis movement.
3. Move planning, need T-shaped movement planning. (Only plan, no pid, pid in downer). Turn maybe need this too. Line following too.
4. Config need to be able to config I2C port what things plugs what port.
5. Can we run the line follower at a lower level controller? Are there any API avaliable for that?
6. I remember there are better and many way to control the chassis for the origional, please list and explain them, and I think we need to use the one that got pid or sync in the backbone. For example, the linefollowing part, what is one motor is faster than the other by the manufature error? Than the pd can never go smooth, how to solve this problem, need u to present solutions. Same as normorl moving and turning.
7. Can we follow the line but backwords?
8. Not really a problem, but <error type> *LineFollowerClass::getTracer(), can we fix the error type while hover?
9. Add a global pwm upper and lowest bound in config.
10. U think we need to create a class for utils? like clamp, round, and other math? This is good for in competition writing code, if I need to do some logic writing, I can modify quicly. We may need a debate on this.
11. void LineFollowerClass::setEnabled(bool on) we need to control brake or not. Because sometimes we want to go further for a little for space for turning. And we don't want to stop. Remember that the path planner need to do this plan (brake or not) too. Write this note to decisions.

# Note for Diego
1. Remember to add the good and what prefer move system into claude.md