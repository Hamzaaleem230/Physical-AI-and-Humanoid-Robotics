import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import PreferenceForm from '../components/PreferenceForm';
import useAuth from '@site/src/hooks/useAuth';
import { useHistory } from '@docusaurus/router';

const PERSONALIZATION_API_BASE_URL = 'https://hamzasyed001122-content-personalization-backend.hf.space/personalization';

function PersonalizationSettingsPage() {
    const { jwtToken } = useAuth();
    const history = useHistory();
    const [preferences, setPreferences] = useState(null);
    const [loading, setLoading] = useState(true);

    const fetchPreferences = async () => {
        const tokenToUse = jwtToken || 'test-token-123';
        setLoading(true);
        try {
            const response = await fetch(`${PERSONALIZATION_API_BASE_URL}/preferences`, {
                headers: { 'Authorization': `Bearer ${tokenToUse}` }
            });
            if (response.ok) {
                const data = await response.json();
                setPreferences(data);
            } else {
                setPreferences(null);
            }
        } catch (e) {
            console.error("Fetch Error:", e);
            setPreferences(null);
        } finally {
            setLoading(false);
        }
    };

    const handleSavePreferences = async (newPreferences) => {
        const tokenToUse = jwtToken || 'test-token-123';
        setLoading(true);
        try {
            const response = await fetch(`${PERSONALIZATION_API_BASE_URL}/preferences`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${tokenToUse}`
                },
                body: JSON.stringify(newPreferences),
            });
            
            if (response.ok) {
                alert('✅ Preferences saved successfully!');
                setTimeout(() => {
                    history.push('/preface'); // Redirect logic
                }, 1500);
            }
        } catch (e) {
            alert(`Error saving preferences: ${e.message}`);
        } finally {
            setLoading(false);
        }
    };

    useEffect(() => {
        fetchPreferences();
    }, [jwtToken]);

    if (loading) {
        return <Layout><div>Loading preferences...</div></Layout>;
    }

    return (
        <Layout title="Personalization Settings">
            {/* Pehli wali original styling: Padding 40px, MaxWidth 800px */}
            <div style={{ padding: '40px', maxWidth: '800px', margin: '0 auto' }}>
                <h1>Personalization Settings</h1>
                <p>Adjust your learning preferences to get tailored content and recommendations.</p>
                <PreferenceForm
                    initialPreferences={preferences}
                    onSave={handleSavePreferences}
                />
            </div>
        </Layout>
    );
}

export default PersonalizationSettingsPage;