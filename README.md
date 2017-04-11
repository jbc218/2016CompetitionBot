# This was the code for our 2016 competition bot
Before you read through the code, I'm so, so, sorry. I didn't use git last year, so all the old code is just commented in. To better understand the functionality, you'd probably have to reference the code in the practice bot code, which I won't ever upload.

Just looking through the code gives me chills, and the only reason I'm reading it is because the mentors actually want me to make it function again for some reason.

This makes the TODO list very very simple.

# TODO
+ Rewrite the entire robot code using the same format as the 2017 robot

# Future of this project
First commit will basically be the same thing except with 2 imports commented out in Robot and 2 lines dealing with the camera in Robot commented out. The first commit basically just makes it so the CANTalons work with the new library.
Next commit will either be an incomplete or complete rewrite of the entire code
Anything after that will either fix issues or add other functionality if needed.

# What will actually be rewritten
First off, FullPowerIntake, IntakeBalance, IntakeDown, LowGoal, Collect, StopPowerToIntake should all be mutable states within a single class with an iterator. Shoot could (hopefully) also be inside that class, but I have to look.
That on its own would fix the stupid button problem that occurs when a Timer.delay() is happening. (when a timer.delay() is on, buttons won't do anything)
I'll also delete all those useless classes like DriveDistance, DriveControllerTurnInPlace, DriveAngle, DriveSpeed, Pose, and also fix Intake.

VisionCV will also be rewritten but I'm not sure what I'll do with it, as we never got auto fully working. Last I remember, all I could do was some weird magic with the arm and intake (Don't ask how I got those timer delays so perfect, I guessed them all right the first time. It moves the arm back and forward to kick the ball back into the intake, pulls it down a specific distance and stops.), then it drives under the low bar and stutters as it tries and fails to turn.

Chances are, it'll be easier to only save tuned values and rewrite the project from scratch, so don't expect the next commit to look anything like this initial one.


#Extra notes

If you have any questions, sorry. My email is jbc218@gmail.com and I'll answer any questions you have. Even if it's 5 years from now, I'll make an effort to get back to you and help you.
