import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI and Humanoid Robotics',
  tagline: 'From First Principles to Real-World Deployment',
  url: 'https://hamzaaleem230.github.io',
  baseUrl: '/',
  projectName: 'Physical-AI-and-Humanoid-Robotics-Book',
  organizationName: 'hamzaaleem230',
  trailingSlash: false,
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  customFields: {
    BACKEND_URL: 'https://hamzasyed001122-rag-chatbot-backend.hf.space',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          editUrl:
            'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics-Book/edit/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI and Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
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
          type: 'html',
          value: '<div id="auth-status"></div>',
          position: 'right',
        },
        {
          href: 'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics-Book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [{ label: 'Start Reading', to: '/preface' }],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Hamzaaleem230/Physical-AI-and-Humanoid-Robotics-Book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Hamza Aleem. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'python', 'cpp', 'matlab', 'json', 'javascript', 'typescript'],
    },
  },
};

export default config;
