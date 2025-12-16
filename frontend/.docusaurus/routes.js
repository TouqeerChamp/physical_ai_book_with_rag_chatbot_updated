import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/physical-ai-book/__docusaurus/debug',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug', '12f'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/config',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/config', '4d3'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/content',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/content', 'a5b'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/globalData',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/globalData', 'abe'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/metadata',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/metadata', '587'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/registry',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/registry', '2ef'),
    exact: true
  },
  {
    path: '/physical-ai-book/__docusaurus/debug/routes',
    component: ComponentCreator('/physical-ai-book/__docusaurus/debug/routes', '1a5'),
    exact: true
  },
  {
    path: '/physical-ai-book/',
    component: ComponentCreator('/physical-ai-book/', '4f9'),
    routes: [
      {
        path: '/physical-ai-book/',
        component: ComponentCreator('/physical-ai-book/', 'd96'),
        routes: [
          {
            path: '/physical-ai-book/',
            component: ComponentCreator('/physical-ai-book/', 'ce3'),
            routes: [
              {
                path: '/physical-ai-book/chapter-02-hardware',
                component: ComponentCreator('/physical-ai-book/chapter-02-hardware', '281'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-03-ros2',
                component: ComponentCreator('/physical-ai-book/chapter-03-ros2', '6bc'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-04-vision',
                component: ComponentCreator('/physical-ai-book/chapter-04-vision', '8d6'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-05-control',
                component: ComponentCreator('/physical-ai-book/chapter-05-control', '12d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-06-ai-integration',
                component: ComponentCreator('/physical-ai-book/chapter-06-ai-integration', 'c4c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-07-navigation',
                component: ComponentCreator('/physical-ai-book/chapter-07-navigation', 'c0a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/chapter-08-ethics',
                component: ComponentCreator('/physical-ai-book/chapter-08-ethics', '000'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/conclusion',
                component: ComponentCreator('/physical-ai-book/conclusion', '779'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/intro',
                component: ComponentCreator('/physical-ai-book/intro', '4fd'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/physical-ai-book/',
                component: ComponentCreator('/physical-ai-book/', '5eb'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
