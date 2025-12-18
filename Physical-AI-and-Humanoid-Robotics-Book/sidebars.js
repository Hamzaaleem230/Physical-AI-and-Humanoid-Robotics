module.exports = {
  humanoidRoboticsSidebar: [
    {
      type: 'doc',
      id: 'preface',
      label: 'Preface: Why Physical AI & Why Humanoids',
    },
    {
      type: 'doc',
      id: 'how-to-use-this-book',
      label: 'How to Use This Book',
    },
    {
      type: 'doc',
      id: 'hardware-requirements',
      label: 'Hardware Requirements Guide',
    },
    {
      type: 'doc',
      id: 'software-stack',
      label: 'Software Stack Overview',
    },

    // MODULE 1 — ROS2 Nervous System
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/ros2-nervous-system/what-is-physical-ai',
        'modules/ros2-nervous-system/ros2-architecture',
        'modules/ros2-nervous-system/building-ros2-packages',
        'modules/ros2-nervous-system/launch-files-parameters-tf2',
        'modules/ros2-nervous-system/urdf-for-humanoids',
        'modules/ros2-nervous-system/motor-commands-sensors-control-loops',
      ],
    },

    // MODULE 2 — DIGITAL TWIN
    {
      type: 'category',
      label: 'Module 2 — The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/digital-twin/gazebo-setup-physics-engine',
        'modules/digital-twin/urdf-to-sdf-conversion',
        'modules/digital-twin/sensor-simulation',
        'modules/digital-twin/collisions-dynamics-balance-simulation',
        'modules/digital-twin/unity-for-high-fidelity-rendering',
        'modules/digital-twin/ros2-integration-with-simulation',
      ],
    },

    // MODULE 3 — ISAAC BRAIN
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac Platform)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/isaac-brain/isaac-sim-omniverse-introduction',
        'modules/isaac-brain/synthetic-data-generation',
        'modules/isaac-brain/isaac-ros-vslam-perception-navigation',
        'modules/isaac-brain/nav2-for-humanoid-locomotion',
        'modules/isaac-brain/reinforcement-learning-for-control',
        'modules/isaac-brain/sim-to-real-transfer-design',
      ],
    },

    // MODULE 4 — VLA
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'modules/vla/whisper-voice-commands-integration',
        'modules/vla/llm-planning-natural-language-to-ros2-tasks',
        'modules/vla/multimodal-interaction',
        'modules/vla/vla-control-graphs-for-humanoids',
        'modules/vla/full-loop-voice-plan-navigate-perceive-manipulate',
      ],
    },

    // Personalization Feature
    {
      type: 'category',
      label: 'Personalization',
      collapsible: true,
      collapsed: false,
      items: [
        {
          type: 'link',
          label: 'Personalization Settings',
          href: '/personalization-settings',
        },
        {
          type: 'link',
          label: 'Recommendations',
          href: '/recommendations',
        },
      ],
    },

    // MINI PROJECTS (EMPTY BUT SAFE)
    {
      type: 'category',
      label: 'Mini-Projects & Capstone',
      collapsible: true,
      collapsed: false,
      link: {
        type: 'generated-index',
        description: 'Mini projects aur capstone modules ka overview.',
      },
      items: [],
    },

    {
      type: 'doc',
      id: 'weekly-roadmap',
      label: 'Weekly Roadmap (Weeks 1-13)',
    },
    {
      type: 'doc',
      id: 'assessment-criteria',
      label: 'Assessment Criteria',
    },
    {
      type: 'doc',
      id: 'glossary',
      label: 'Glossary of Robotics Terms',
    },
    {
      type: 'doc',
      id: 'references',
      label: 'References & Further Reading',
    },
  ],
};
