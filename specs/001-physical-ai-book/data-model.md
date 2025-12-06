# Data Model

## Book

- **`title`**: String
- **`version`**: String
- **`audience`**: String
- **`theme`**: String
- **`goal`**: String
- **`modules`**: Array<Module>
- **`frontmatter`**: Array<Frontmatter>
- **`backmatter`**: Array<Backmatter>

## Module

- **`id`**: String
- **`title`**: String
- **`goal`**: String
- **`chapters`**: Array<Chapter>

## Chapter

- **`title`**: String
- **`content`**: String

## Frontmatter

- **`title`**: String
- **`content`**: String

## Backmatter

- **`title`**: String
- **`content`**: String

## Humanoid Robot

- **`name`**: String
- **`urdf`**: String (Path to URDF file)
- **`sensors`**: Array<Sensor>
- **`actuators`**: Array<Actuator>

## Sensor

- **`name`**: String
- **`type`**: String (e.g., LiDAR, IMU, Depth Camera)
- **`topic`**: String (ROS 2 topic)

## Actuator

- **`name`**: String
- **`type`**: String (e.g., Motor)
- **`topic`**: String (ROS 2 topic)
