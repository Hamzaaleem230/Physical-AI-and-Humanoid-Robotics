import React, { useState, useEffect } from 'react';

function RecommendationPanel() {
    const [recommendations, setRecommendations] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        const fetchRecommendations = async () => {
            setLoading(true);
            setError(null);
            try {
                // Placeholder for actual API calls
                // const response = await fetch('/personalization/recommendations', {
                //     headers: {
                //         'Authorization': `Bearer ${YOUR_JWT_TOKEN}`
                //     }
                // });
                // if (!response.ok) {
                //     throw new Error(`HTTP error! status: ${response.status}`);
                // }
                // const data = await response.json();
                // setRecommendations(data.recommendations);

                // Mocking API call for demonstration
                await new Promise(resolve => setTimeout(resolve, 700));
                setRecommendations([
                    { contentId: "ros2-nodes", title: "Understanding ROS 2 Nodes" },
                    { contentId: "isaac-sim-omniverse-introduction", title: "Introduction to Isaac Sim & Omniverse" },
                    { contentId: "urdf-for-humanoids", title: "URDF for Humanoid Robots" },
                ]);
            } catch (e) {
                setError(e.message);
            } finally {
                setLoading(false);
            }
        };

        fetchRecommendations();
    }, []);

    if (loading) {
        return <div className="recommendation-panel">Loading recommendations...</div>;
    }

    if (error) {
        return <div className="recommendation-panel">Error loading recommendations: {error}</div>;
    }

    if (recommendations.length === 0) {
        return <div className="recommendation-panel">No recommendations available at this time.</div>;
    }

    return (
        <div className="recommendation-panel">
            <h3>Recommended for You</h3>
            <ul>
                {recommendations.map(item => (
                    <li key={item.contentId}>
                        <a href={`/docs/${item.contentId}`}>{item.title}</a> {/* Assuming Docusaurus doc link structure */}
                    </li>
                ))}
            </ul>
        </div>
    );
}

export default RecommendationPanel;
