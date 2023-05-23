# 2023_robot_code_py

Code for our 2023 Robot Code, Bender, but written in Python!

---

In summary, I wanted to try to rewrite our robot code from this year in Python, to see how it worked in comparison to our original language, Java. Of course, I am kind of bias towards Python since it's one of my favorite langauges, but I suggest using it in the future.

python: https://python.org

robotpy: https://robotpy.readthedocs.io/en/stable/

Java robot code: https://github.com/team178/2023RobotCode

In this code, I took some time to try and make things a little more pythonic, but I still ended up giving in and using some more of the same systems, such as commands and subsystems because they work well for autonomous routes. But, I made some utilities and some structures that are more Python-like, which make it easier to have more readable  and clean looking code.

Most of the code is documented in-place, so the code itself will have docstrings or comments explaining things that might not make sense at first glance.

## Project Structure

### Packages

Python code, and `robotpy` robot code, can be written using just a `robot.py` file and the Python interpreter. You don't need as much stuff as I have here. But, I like to structure my Python projects using what are called packages to keep my code organized.

Packages in python are basically just folders that contain modules, which are just .py files. For this project, all of our robot code is contained under the `robot` package. This means that from any module in that package, I can use an absolute import when I want to import something from another file, allowing less errors when trying to get things to run.

For example, you could have a `robot.py` and a `drivetrain.py` file in a folder:

```py
# robot.py
from drivetrain import Drivetrain

class Robot:
    ...

# drivetrain.py

class Drivetrain:
    ...
```

This import, `drivetrain` is relative. It's searching relative to the file/where the interpreter is run from, I can't remember exactly which, for a module named `drivetrain`. If it can't find one, it searches your installed third-party packages. This means that if something weird happens, and your code is run from another folder, then sometimes it has trouble locating your module. I've had many problems with this in the past.

But, if we use a package, like `robot`, we can make it so that `robot.py` knows exactly where `drivetrain.py` is, beacuse they are both under the same namespace.

```py
# robot/robot.py
from robot.drivetrain import Drivetrain

class Robot:
    ...

# robot/drivetrain.py

class Drivetrain:
    ...
```

Then, a single entry point from outside of the module is used to enter it, and from there, every import is under the `robot` package, and there is no confusion in the interpreter.

This also allows just for general organization, with subpackages being used to organize things like subsystems, commands, and utilities into their own sub-packages.

### Package strangeness

Beacuse of how robotpy is written, my package structure is not really supported without a little bit of hacking.

To fix this, I made the `entry.py`. More about this and what `RobotEntry` does is written in this file.

### pdm

A package/dependency manager is a very useful utility. Java has a few, with the one we use being Gradle. Gradle manages all third party dependencies of a project, and is responsible for downloading, installing, and running your code with the correct versions of third-party software, such as WPILib.

Python has a package manager called `pip`. It can be used to install and manage third party packages from https://pypi.com. But unfortunately, it has some setbacks.

Many other package managers have a per-project configuration. The dependencies for that project are defined in a config file specifically for that project, and this prevents whatever packages you install for one project to not interfere with another. It also allows for the same packages to be installed on other machines without having to manually go and figure out versions.

`pip` is not very good at this. The best it can do is install all of your packages on your computer globally, and the most per-project configuration it can do is create a `requirements.txt` file, that lists the packages and versions you need. This can then be used to install them globally.

So, for this project, I'm using a third-party dependency manager called `pdm` (https://github.com/pdm-project/pdm). Looking in the `pyproject.toml` file, you can see a list of dependencies:

```toml
dependencies = [
    "robotpy>=2023.4.3.0",
    "robotpy-ctre>=2023.1.0",
    "robotpy-rev>=2023.1.3.2",
    "robotpy-commands-v2>=2023.4.3.0",
    "robotpy-pathplannerlib>=2023.3.4.1",
]

[tool.pdm]
[tool.pdm.dev-dependencies]
dev = [
    "black>=23.3.0",
    "setuptools>=67.6.1",
]
```

This list of dependencies is all of the packages and the versions of those packages needed for this project only. They can be installed *for this project* using the `pdm install` command. These installed packages are installed in a virtual enviroment, which is a python interpreter and package directory seperate from the global one. Any packages installed for this project do not interfere with other projects and their packages on your computer.

`pdm` also allows for creating scripts, such as these below:

```toml
[tool.pdm.scripts]
robot = "python entry.py"
sim = "python entry.py sim"
deploy = "python entry.py deploy --nonstandard"
format = "black ."
```

If I want to deploy my code, instead of typing `python entry.py deploy --nonstandard` every time I want to deploy, I can just do `pdm deploy`. This is similar to Gradle's `./gradlew deploy` or `./gradlew build`.

Overall, I recommend using `pdm` to manage this project. Here's a cheatsheet:

```zsh
# install pdm
python -m pip install pdm

# install dependencies and create a venv
pdm install

# simulate robot code using robot sim
pdm sim

# deploy robot code to a real robot
pdm deploy
```

### Physics

`robotpy` has some really nice support for physics. All physics code can be place in `physics.py`, and it is run whenever robot sim is launched. If you really wanted to expand and abstract this physics feature you could, but it seems simple enough just to leave everything in the one file.

The only physics I've implemented for this code is the arm, but drivetrain physics can be added as well.

## Styling

Unlike Java, Python has an official style guide. For Java, I still can't really find an "official" one. Many different companies and groups have their own systems for naming and organizing. Python has an official one, called PEP 8 (https://peps.python.org/pep-0008/).

I tried to stick with PEP 8's naming conventions as much as possible, using snake_case for eveything except for class names (these are in PascalCase, like Java), and using SCREAMING_SNAKE_CASE for constants. The only things that use camelCase are methods of things from robotpy, where it simply wraps the underlying C++ function, named in camelCase.

To keep things neat, I used a formatting tool called `black`, which enforces and reformats your code to adhere to PEP 8 with a snigle command. I setup a quick script to do this, which is `pdm format`. You might also see comments that say `# fmt: off/on`. These tell the formatter when it should ignore a block of code. This is used sometimes because sometimes it formats the code in a way that isn't as readable as it thinks it is.

I also took the time to include typehints. In most langauges, you need to declare the type of a variable. But in Python, variables are dynamic, and can contain any value at any time. To make it easier to know what type a variable is, or what type of data a function/class might take as an argument or return, typehints are used to hint at what the type used is.

For example, our `mirror_trajectory` function:
```py
def mirror_trajectory(traj: Trajectory) -> Trajectory:
    ...
```

This function's typehints say that it's parameter, `traj`, should be a `Trajectory`, and that it returns a `Trajectory`. We could also just not have the typehints that would result in something like this:

```py
def mirror_trajectory(traj):
    ...
```

But, these typehints are useful. Python does not enforce them, so this function could return a float instead of a `Trajectory` and it would not care, but that's why they're called type*hints*.

## Decorators

The most common one you'll probably see is decorators. Decorators in Python allow you to decorator, or wrap functions with additional functionality. They appear above functon definitions with an @ symbol, like so:

```py
    @cmd.run
    def drive_until_level(self, speed: float):
        """
        Drive at the provided `speed` until the robot is leveled.
        Negative speed is backwards, positive forwards.
        """
        if -8 > self.get_level_heading():
            self._arcade(speed, 0)
        elif self.get_level_heading() > 8:
            self._arcade(-speed, 0)
        else:
            self._arcade(0, 0)
```
But, lets see what it would look like without the decorator:

```py
    def drive_until_level(self, speed: float):
        def _command():
            if -8 > self.get_level_heading():
                self._arcade(speed, 0)
            elif self.get_level_heading() > 8:
                self._arcade(-speed, 0)
            else:
                self._arcade(0, 0)
        
        return commands2.cmd.run(_command)
```

This doesn't look as clean. In Java, this code would have looked like this:

```java
public static Command driveUntilLevel(double speed) {
    this.run(() -> {
        if (-8 > m_drivetrain.getLevelHeading()) {
            m_drivetrain.arcadeDrive(-speed, 0);
        } else if (m_drivetrain.getLevelHeading() > 8) {
            m_drivetrain.arcadeDrive(speed, 0);
        } else {
            m_drivetrain.setWheelSpeeds(0, 0);
        }
    })
}
```

Using the decorator, we move the extra boilerplate for the command to somewhere outside of the main body of the function, into a nice little tag above it that tells us it's a `run` command, and that whatever code inside of it is only the code executed during that Command.

You can see the code behind the decorator in `utils/cmd.py`.

I also used decorators in the `AutoSelector`. Of course, I could have just created all of my commands, and then hard coded them into a `SendableChooser`, like this:
```py
chooser.addOption("PlaceHigh", place_high)
chooser.addOption("MidCubeBalance", cube_balance)
...
```
But, I wanted to make it more expandable. So, I created the `AutoSelector.route` decorator. `AutoSelector` is a singleton, so there will only ever be one instance of it. It can be imported anywhere, and it will still be the same instance of the object.

This `auto.route` command takes a function that returns a `Command`, and adds it to the auto selector. These can all be seen in the `robot/auto/routes.py` file. Here's our 'place high' command, for example:

```py
@auto.route("PlaceHigh")
def place_high(robot) -> Command:
    return sequence(
        robot.claw.close(),
        robot.arm.set_position(ArmPosition.HIGH),
        WaitCommand(1.5),
        robot.claw.open(),
        WaitCommand(0.3),
        robot.arm.set_position(ArmPosition.HOME),
    )
```

Any function that returns a valid command can have the `auto.route()` decorator added to it, and it will be shown in the selector. This makes it easy to create a new command, while only adding to one part of the code, instead of having to go back and forth through the code. Everything about a command is in one place.

Feel free to use these same systems in other code, or modify them to work for you.

## Downsides of Python

Just like every programming langauge, there are weaknesses:

- Python is... slow?
  - Python isn't the fastest langauge on the planet. But, I say this first because it doesn't really matter. It's been a long debate as to wether C++ or Java is better for FRC, with some arguments saying that C++ is faster so it's better. But speed isn't always everything, and it's been found it doesn't really matter what language you use for FRC. Python is a bit slower compared to compiled langauges, but it's fast enough to work very well in FRC. I think Python is a perfect language for FRC, because it allows for a lot more readability and functionality that is easier for beginners to understand the concepts of programming.

- It's interpreted
  - The major difference between C++ and Java is that Python is not compiled, it's interpreted. There is not 'build' or 'compile' stage of running a script. It reads the code line by line, and executes it once it gets to each line. This saves you from waiting for the build stage, but it also means that any errors you would get at compile time in any other language appear at *runtime*, including syntax errors. This means that sometimes, you won't find a syntax error until the robot tries to run the code that has the syntax error. However, this can be minimized by paying attention to your editor, which will notifiy you of many errors before running/deploying your code.