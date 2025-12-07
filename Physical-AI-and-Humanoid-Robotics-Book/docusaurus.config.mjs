import { themes as prismThemes } from 'prism-react-renderer';
import path from 'path';

export default {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'From First Principles to Real-World Deployment',
  url: 'https://Hamzaaleem230.github.io',
  baseUrl: '/', 
  onBrokenLinks: 'throw', 
  favicon: 'img/favicon.ico',
  organizationName: 'Hamzaaleem230',
  projectName: 'Physical-AI-and-Humanoid-Robotics-Book',

  // 🌟 FIX: DEPRECATED 'env' field ko 'customFields' mein move kar diya gaya hai.
  // Access: client-side code mein ab yeh 'siteConfig.customFields.BACKEND_URL' ke zariye accessible hoga.
  customFields: {
    BACKEND_URL: process.env.BACKEND_URL || '/api',
  },
  // ----------------------------------------------------------------------------------
  
  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: path.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics',
          routeBasePath: '/',
        },
        blog: {
          showReadingTime: true,
          editUrl:
            'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics',
        },
        theme: {
          customCss: path.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  markdown: {
    hooks: {
      // onBrokenMarkdownLinks ko yahan 'markdown.hooks' ke andar rakha gaya hai
      onBrokenMarkdownLinks: 'warn', 
    },
  },

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    navbar: {
      title: 'Physical AI and Humanoid Robotics',
      logo: {
        alt: 'My Project Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'humanoidRoboticsSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs', 
          items: [{ label: 'Book', to: '/preface' }],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/docusaurus',
            },
            { label: 'Discord', href: 'https://discordapp.com/invite/docusaurus' },
            { label: 'Twitter', href: 'https://twitter.com/docusaurus' },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics/tree/main',
            },
          ],
        },
      ],
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};