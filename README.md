
# Robotics Studio Code Standard
The code standards that we will be using to ensure clean and legible code.

### Variables
- **Style:** CamelCase
- **Example:**
  ```cpp
  int counter = 0;
  int startCount = 0;
  ```
- **Avoid:** 
  ```cpp
  int start_count = 0;
  int startcount = 0;
  ```

### Private Variables
- Follows the same CamelCase convention as regular variables.

### Functions
- **Style:** CamelCase
- **Example:**
  ```cpp
  void initializeRobot();
  void calculateTrajectory();
  ```

### Classes
- **Style:** PascalCase
- **Example:**
  ```cpp
  NavigationNode node;
  PathPlanner planner;
  ```

## Documentation

### Node Creation
- The main function and class should be included in the same file to reduce the number of files needed for node creation.

### Comments and Redundancy
- Comments should be included for each variable to explain its purpose.
- Remove redundant variables to streamline the code.

### Doxygen
- **Code Documentation:** All code functionality should be documented using Doxygen.
- **Usage:** Doxygen comments should be placed above each class, function, and variable to ensure that the generated documentation is comprehensive.

## Additional Guidelines

- **Simplistic Naming:** Names should be kept as simple as possible, avoiding underscores or other special characters.

Thank you for adhering to these standards to ensure that our codebase remains clean, consistent, and easy to navigate.
```

This should be a clean and readable Markdown version of your original content.
