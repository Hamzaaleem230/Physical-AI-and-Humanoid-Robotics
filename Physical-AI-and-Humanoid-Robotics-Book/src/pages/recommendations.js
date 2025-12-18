import React from 'react';
import Layout from '@theme/Layout'; // Layout import karna zaroori hai
import RecommendationPanel from '../components/RecommendationPanel';

function RecommendationsPage() {
    return (
        <Layout title="Your Recommendations">
            <div style={{ padding: '40px', maxWidth: '800px', margin: '0 auto' }}>
                <h1>Personalized Recommendations</h1>
                <p>Tailored content based on your profile and reading habits.</p>
                <hr style={{ margin: '20px 0' }} />
                <RecommendationPanel />
            </div>
        </Layout>
    );
}

export default RecommendationsPage;