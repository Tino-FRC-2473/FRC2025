# FRC 2025

## Code Structure
This project will use the FSMSystem framework, representing each subsystem as a finite state machine.

FSM diagrams for each subsystem are located in the [doc](doc) folder.
* [Drive FSM](doc/DriveFSM.md)
* [Elevator FSM](doc/ElevatorFSM.md)
* [Funnel FSM](doc/FunnelFSM.md)
* [Climber FSM](doc/ClimberFSM.md)

[![Javadoc](https://github.com/Tino-FRC-2473/FRC2025/actions/workflows/javadoc.yml/badge.svg)](https://github.com/Tino-FRC-2473/FRC2025/actions/workflows/javadoc.yml)

Javadoc for this repo is available at [https://tino-frc-2473.github.io/FRC2025/](https://tino-frc-2473.github.io/FRC2025/)

## Code Conventions
We will base our code style off the [Sun Java style guide](https://www.oracle.com/technetwork/java/codeconventions-150003.pdf).
 * Indentation: tabs
 * Braces: endline
 * Wrap lines at 80 characters

Some additional naming guidance:
 * Variable names: camelCase
 * Booleans should start with "is" or "has" (ex. hasMotor, isPositive)

## Commit messages
Everyone should read and follow the rules in "[How to write a Git Commit Message](https://chris.beams.io/posts/git-commit/)."

# Integration Plan
- Local branches are personal development branches for work in progress code.
	- Naming convention: `dev/week-#/firstname_lastname/description-as-needed`

- Main branch will be where all subsystems come together.
	- Main branch merges require formal pull requests (peer & mentor reviewed) and must pass build and style checks. These changes should already have been tested on a test branch.
	- Main branch will serve as the weekly baseline for new development.
  - Main branch is expected to be configured to run on the competition robot.

# Competition branching
A dedicated branch will be created to track code changes made at each competition.
- For Pinnacles, the competition branch will be `comp/pinnacles`.
- All changes must be commited to this branch prior to the final robot code upload before queueing for each match.

The commit used for each match must be tagged prior to queueing. This is essential for post-match debugging and analysis. Tags should be namespaced under the competition name and indicated the match type `qual` or `playoff`.
- For instance, qualification match 3 at Pinnacles should be tagged with `comp/pinnacles/qual_3`.
- Create annotated tags with the `git tag` command. I.e. `git tag -a "comp/pinnacles/qual_3"`. Write a summary of any code or configuration changes made in the tag description.
- Tags must be pushed with `--tags` option. I.e `git push origin --tags`.
