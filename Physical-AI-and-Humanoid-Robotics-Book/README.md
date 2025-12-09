# Physical AI and Humanoid Robotics Book

This repository contains the complete content for the "Physical AI and Humanoid Robotics" book, authored by the Gemini CLI. The book is structured and formatted for deployment using Docusaurus, a static site generator.

## Book Structure

The book is organized into four main modules, each covering a crucial aspect of Physical AI and Humanoid Robotics:

*   **Module 1 — The Robotic Nervous System (ROS 2):** Fundamentals of ROS 2 for communication and control.
*   **Module 2 — The Digital Twin (Gazebo & Unity):** High-fidelity simulation environments for prototyping and testing.
*   **Module 3 — The AI-Robot Brain (NVIDIA Isaac Platform):** Leveraging NVIDIA's ecosystem for advanced AI perception, navigation, and reinforcement learning.
*   **Module 4 — Vision-Language-Action (VLA):** Integrating LLMs for natural language understanding and high-level robot planning.

Each module includes detailed chapters, code examples, mini-projects, and a capstone project to provide a comprehensive learning experience.

## Getting Started with the Book (Local Development)

To run this book locally using Docusaurus, follow these steps:

### 1. Prerequisites

Ensure you have Node.js (v18 or higher recommended) and npm (Node Package Manager) installed on your system.

```bash
node -v
npm -v
```

If not installed, download and install from [nodejs.org](https://nodejs.org/).

### 2. Install Docusaurus Dependencies

Navigate to the `docs` directory and install the necessary npm packages:

```bash
cd docs
npm install
```

### 3. Run the Docusaurus Development Server

After installing dependencies, you can start the local development server:

```bash
npm run start
```

This command starts a local development server and opens a browser window. Most changes are reflected live without restarting the server. The book will typically be available at `http://localhost:3000`.

### 4. Build the Static Website

To build the static HTML, CSS, and JavaScript files for production deployment:

```bash
npm run build
```

This command generates static content into the `build` directory.

## Deployment Notes

This book is configured for deployment to GitHub Pages via a CI/CD pipeline. The workflow is defined in `.github/workflows/main.yml`.

To deploy, ensure:
*   Your repository is hosted on GitHub.
*   The `main` branch is configured as the source for GitHub Pages in your repository settings.
*   The GitHub Actions workflow has appropriate permissions to publish to GitHub Pages.

## Repository Layout

*   `/docs/`: The Docusaurus project root, containing all book content in Markdown/MDX.
*   `/docs/sidebars.js`: Defines the book's navigation structure.
*   `/docs/modules/`: Contains the chapters for each of the four modules.
*   `/examples/`: Contains code examples and mini-projects for each module.
*   `/assets/`: Static assets such as images and diagrams used throughout the book.
*   `/robots/`: URDF and other robot-specific description files.
*   `/sim/`: Simulation worlds and configurations for Gazebo and Isaac Sim.
*   `/ci/`: CI/CD pipeline configurations (e.g., GitHub Actions workflows).
*   `/specs/`: Feature specifications for the project.

---

We hope you enjoy your journey into "Physical AI and Humanoid Robotics"!
