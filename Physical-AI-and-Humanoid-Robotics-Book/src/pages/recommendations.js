import React from 'react';
import Layout from '@theme/Layout';
import RecommendationPanel from '../components/RecommendationPanel';
// 🔹 Docusaurus ka BrowserOnly import karein
import BrowserOnly from '@docusaurus/BrowserOnly';

function RecommendationsPage() {
    return (
        <Layout title="Your Recommendations">
            <div style={{ padding: '40px', maxWidth: '800px', margin: '0 auto' }}>
                <h1>Personalized Recommendations</h1>
                <p>Tailored content based on your profile and reading habits.</p>
                <hr style={{ margin: '20px 0' }} />
                
                {/* 🔹 Isay BrowserOnly mein wrap kar dein taake build crash na ho */}
                <BrowserOnly fallback={<div>Loading your personalized content...</div>}>
                    {() => <RecommendationPanel />}
                </BrowserOnly>
            </div>
        </Layout>
    );
}

export default RecommendationsPage;