
# Robotics Studio Code Standard

Welcome to the Robotics Studio project! This document outlines the coding standards and naming conventions that we follow to maintain clean, consistent, and easily understandable code.

## Naming Conventions

### Variables
- **Style:** CamelCase
- **Example:**
  ```cpp
  int counter = 0;
  int startCount = 0;
  ```
Avoid: 
  ```cpp
int start_count = 0;
int startcount = 0;
```
Private Variables
Follows the same CamelCase convention as regular variables.
Functions
Style: CamelCase
Example:
void initializeRobot();
void calculateTrajectory();

Classes
Style: PascalCase
Example:
NavigationNode node;
PathPlanner planner;

Documentation
Node Creation
The main function and class should be included in the same file to reduce the number of files needed for node creation.
Comments and Redundancy
Comments should be included for each variable to explain its purpose.
Remove redundant variables to streamline the code.
Doxygen
Code Documentation: All code functionality should be documented using Doxygen.
Usage: Doxygen comments should be placed above each class, function, and variable to ensure that the generated documentation is comprehensive.
Additional Guidelines
Simplistic Naming: Names should be kept as simple as possible, avoiding underscores or other special characters.
Thank you for adhering to these standards to ensure that our codebase remains clean, consistent, and easy to navigate.
