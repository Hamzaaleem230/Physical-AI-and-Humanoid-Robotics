import React, { useState, useEffect } from 'react';
import PreferenceForm from '../components/PreferenceForm';

function PersonalizationSettingsPage() {
    const [preferences, setPreferences] = useState(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    // Placeholder for actual API calls
    const fetchPreferences = async () => {
        setLoading(true);
        setError(null);
        try {
            // In a real app, you'd fetch from your backend API
            // const response = await fetch('/personalization/preferences', {
            //     headers: {
            //         'Authorization': `Bearer ${YOUR_JWT_TOKEN}`
            //     }
            // });
            // if (!response.ok) {
            //     throw new Error(`HTTP error! status: ${response.status}`);
            // }
            // const data = await response.json();
            // setPreferences(data);

            // Mocking API call for demonstration
            await new Promise(resolve => setTimeout(resolve, 500));
            setPreferences({
                skillLevel: 'Intermediate',
                interests: ['ROS2', 'Computer Vision'],
                learningStyle: 'Practical',
            });
        } catch (e) {
            setError(e.message);
        } finally {
            setLoading(false);
        }
    };

    const handleSavePreferences = async (newPreferences) => {
        setLoading(true);
        setError(null);
        try {
            // In a real app, you'd POST to your backend API
            // const response = await fetch('/personalization/preferences', {
            //     method: 'POST',
            //     headers: {
            //         'Content-Type': 'application/json',
            //         'Authorization': `Bearer ${YOUR_JWT_TOKEN}`
            //     },
            //     body: JSON.stringify(newPreferences),
            // });
            // if (!response.ok) {
            //     throw new Error(`HTTP error! status: ${response.status}`);
            // }
            // const data = await response.json();
            // setPreferences(data);
            alert('Preferences saved successfully! (Mocked)');

            // Mocking API call for demonstration
            await new Promise(resolve => setTimeout(resolve, 500));
            setPreferences(newPreferences);

        } catch (e) {
            setError(e.message);
            alert(`Error saving preferences: ${e.message}`);
        } finally {
            setLoading(false);
        }
    };

    useEffect(() => {
        fetchPreferences();
    }, []);

    if (loading) {
        return <div>Loading preferences...</div>;
    }

    if (error) {
        return <div>Error: {error}</div>;
    }

    return (
        <div style={{ padding: '20px', maxWidth: '800px', margin: '0 auto' }}>
            <h1>Personalization Settings</h1>
            <p>Adjust your learning preferences to get tailored content and recommendations.</p>
            <PreferenceForm
                initialPreferences={preferences}
                onSave={handleSavePreferences}
            />
        </div>
    );
}

export default PersonalizationSettingsPage;

// Instructions for Docusaurus integration:
// To integrate this page into your Docusaurus application,
// copy this file to `Physical-AI-and-Humanoid-Robotics-Book/src/pages/PersonalizationSettingsPage.js`
// (or a suitable location within `src/pages`).
//
// Then, you can link to it from your Docusaurus sidebar or navigation.
// Example for `docusaurus.config.js` or `sidebars.js`:
// {
//   type: 'link',
//   label: 'Personalization Settings',
//   to: '/personalization-settings', // This will map to src/pages/PersonalizationSettingsPage.js
// },
