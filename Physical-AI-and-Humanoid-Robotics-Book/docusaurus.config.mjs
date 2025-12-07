import { themes as prismThemes } from 'prism-react-renderer';
import path from 'path';

export default {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'From First Principles to Real-World Deployment',
  url: 'https://Hamzaaleem230.github.io',
  baseUrl: '/', // Broken Links ko theek kiya, ab yeh 'throw' hi rahega (Recommended) // agar aap isse 'warn' ya 'ignore' karna chahte hain toh 'warn' ya 'ignore' likh dein
  onBrokenLinks: 'throw', // ⚠️ DEPRECATED OPTION HATA DI GAYI HAI. Ab yeh 'markdown' object mein hai. // onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',
  organizationName: 'Hamzaaleem230',
  projectName: 'Physical-AI-and-Humanoid-Robotics-Book',

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: path.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics/', // Yeh setting home page ko docs/intro se link karne ke liye zaroori hai
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
  ], // Nayi Markdown Configuration Yahan Add Ki Gayi Hai

  markdown: {
    hooks: {
      // Deprecated option ko yahan move kar diya gaya hai
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
        // { to: '/blog', label: 'Blog', position: 'left' },
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
          title: 'Docs', // Yeh link theek kiya gaya hai taaki yeh 'intro' page ko link kare
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
            // { label: 'Blog', to: '/blog' },
            {
              label: 'GitHub',
              href: 'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics/tree/main/Physical-AI-and-Humanoid-Robotics-Book',
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
