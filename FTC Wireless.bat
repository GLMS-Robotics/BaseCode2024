@echo off
where /q adb
IF ERRORLEVEL 1 (
    color 06
    echo Couldn't find the adb.exe program in your computer's PATH variable!
    echo You need to tell Windows where the adb.exe program installed with Android Studio is.
    echo To do so, first find the program in your files. For me it was in the folder C:\Users\Sam\AppData\Local\Android\Sdk\platform-tools
    echo Once you find the program, open your computer's Environment Variable settings.
    echo Edit the system variable called PATH, and add the folder you found the program in to the list.
    echo Once you save your settings, reopen this file and if everything works, it will try to connect to the robot.
    echo Happy coding!
    pause
) ELSE (
    echo Connecting debugging bridge to robot...
    echo This will allow code to be loaded wirelessly.
    echo Press Enter when you are done to disconnect from the robot.
    echo WARNING: If you disconnect from the robot's wifi before hitting enter here, you will need to restart the robot before you can wirelessly load code again!
    color 0A
    adb.exe connect 192.168.43.1:5555
    pause
    adb.exe disconnect
    pause
)
