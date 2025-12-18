import React from 'react';
import RecommendationPanel from '../components/RecommendationPanel';
import withPersonalization from '../hocs/withPersonalization'; // Import the HOC if personalization context is needed

// This is a demonstration page for the RecommendationPanel component.
// In a real Docusaurus application, you would integrate RecommendationPanel
// into existing content pages or a dedicated recommendations section.

function RecommendationsPage() {
    return (
        <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
            <h1>Personalized Recommendations</h1>
            <p>Here are some content recommendations tailored to your profile.</p>
            <RecommendationPanel />
        </div>
    );
}

// Optionally, you might wrap this page with withPersonalization HOC
// if the recommendations themselves need to adapt based on user preferences,
// or if you want to ensure preferences are loaded for other elements on this page.
// export default withPersonalization(RecommendationsPage);

export default RecommendationsPage;


// Instructions for Docusaurus integration:
// To integrate the RecommendationPanel into your Docusaurus application,
// you can either use this RecommendationsPage.js as a standalone page:
//   - Copy this file to `Physical-AI-and-Humanoid-Robotics-Book/src/pages/RecommendationsPage.js`
//     (or a suitable location within `src/pages`).
//   - Link to it from your Docusaurus sidebar or navigation.
//     Example for `docusaurus.config.js` or `sidebars.js`:
//     {
//       type: 'link',
//       label: 'Recommendations',
//       to: '/recommendations', // This will map to src/pages/RecommendationsPage.js
//     },
//
// OR, you can integrate the <RecommendationPanel /> component directly
// into any existing Docusaurus page or component where you want recommendations to appear.
//
// Example of direct integration in an existing Docusaurus MDX page:
//
// ---
// title: My Awesome Doc
// ---
//
// # My Awesome Doc
//
// Lorem ipsum dolor sit amet...
//
// <RecommendationPanel />
//
// More content...
//
// (Note: To use React components in MDX, you might need to ensure
// React is in scope, or configure Docusaurus MDX options.)
