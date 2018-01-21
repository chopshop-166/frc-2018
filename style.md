# Style Guide
Using a consistent style improves readability and makes it easier to maintain a code base. We will follow the [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html). The important parts of the guide are included below for ease of use. 

## Order
There are a number of different elements found in each file. By grouping these elements together in accordance with their functionality we can ensure that another programmer can quickly gain an understanding of our code. To this end, each source file will be ordered as follows:
1. Imports
1. Class declaration
1. Properties (Public data with get and set functions)
1. Fields (Private data)
1. Constructors
1. Methods (Any helper functions)
1. Commands (Subsystem commands)

Additionally, all variables and methods should be defined before they are first accessed or called. 

## Capitalization
Camel Case is a way of writing names where the first letter of each word is capitalized. There are two forms of Camel Case, Upper and Lower.

* **Upper Camel Case**: The first letter of every word is capitalized
* **Lower Camel Case**: The first letter of every word except the first is capitalized


Use Case | Description | Capitalization | Example
---------|-------------|----------------|---------
Class | Nouns | Uppercase Camel | ButtonJoystick
Method Names | Verbs | Lower Camel | getY
Local Variables | Descriptive Names | Lower Camel | gyroAngle
Constant values | Separated by underscores | Capitalization | GYRO_PORT

## Braces

Braces are used even when optional such as in `if`, `else `, `for`, etc.

* No line breaks before opening brace
* Line break after the opening brace
* Line break before the closing brace
* Line break after the closing brace unless it is followed by an `else`

### Example

```java
if (x==1) {
}
```

## Indentation
*Note that this differs from the Google Java Style Guide.*

All code blocks shall be indented with 4 spaces.

## Javadoc
Javadocs are a standard way of writing comments that explain methods. Every public method should have a Javadoc that explains the function's purpose, parameters, and return value. 

### Example
```java
/**
 * Get a button from this joystick
 * <p>
 * Returns the sepcified button of a joystick without having to explicitely create each button.
* 
* @param buttonId The index of the button to accesss
* @return The button object for the given ID
*/
public Button getButton(int buttonId) {
    return buttons.get(buttonId);
}
```

