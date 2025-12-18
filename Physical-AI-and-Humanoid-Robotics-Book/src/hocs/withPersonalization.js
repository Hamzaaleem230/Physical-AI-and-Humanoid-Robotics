import React, { useState, useEffect, useMemo } from 'react';
import useAuth from '@site/src/hooks/useAuth'; 

// --- FIXED URL ---
// Base URL ko endpoint ke baghair rakhein taake niche fetch asaan ho
const PERSONALIZATION_API_BASE_URL = 'https://hamzasyed001122-content-personalization-backend.hf.space/personalization';

const withPersonalization = (WrappedComponent) => {
    return function PersonalizableComponent(props) {
        const { jwtToken } = useAuth(); 
        const [userPreferences, setUserPreferences] = useState(null);
        const [loadingPreferences, setLoadingPreferences] = useState(true);
        const [preferencesError, setPreferencesError] = useState(null);

        useEffect(() => {
            const fetchUserPreferences = async () => {
                // Testing ke liye: Agar token nahi hai to hum local storage ya dummy token use kar sakte hain
                // Filhal bypass enabled hai backend par, to 'test-token' bhi chalega
                const tokenToUse = jwtToken || 'test-token-123'; 

                setLoadingPreferences(true);
                setPreferencesError(null);
                try {
                    // Sahi Endpoint: BASE_URL + /preferences
                    const response = await fetch(`${PERSONALIZATION_API_BASE_URL}/preferences`, {
                        method: 'GET',
                        headers: {
                            'Authorization': `Bearer ${tokenToUse}`,
                            'Content-Type': 'application/json'
                        }
                    });

                    if (!response.ok) {
                        if (response.status === 404) {
                            // Agar user naya hai aur DB mein data nahi hai
                            setUserPreferences({ skillLevel: 'Beginner', interests: [], learningStyle: 'Theoretical' });
                        } else {
                            throw new Error(`Failed to fetch: ${response.status}`);
                        }
                    } else {
                        const data = await response.json();
                        setUserPreferences(data);
                    }
                } catch (error) {
                    console.error("Personalization Error:", error);
                    setPreferencesError(error.message);
                } finally {
                    setLoadingPreferences(false);
                }
            };

            fetchUserPreferences();
        }, [jwtToken]); 

        const personalizationUtils = useMemo(() => {
            if (!userPreferences) return {};

            const skillLevelOrder = { 'Beginner': 0, 'Intermediate': 1, 'Expert': 2 };

            const shouldShowContent = (minSkillLevel) => {
                if (!userPreferences?.skillLevel) return true;
                const userLevel = skillLevelOrder[userPreferences.skillLevel] || 0;
                const requiredLevel = skillLevelOrder[minSkillLevel] || 0;
                return userLevel >= requiredLevel;
            };

            const adaptText = (text, theoreticalText, practicalText) => {
                if (!userPreferences?.learningStyle) return text;
                if (userPreferences.learningStyle === 'Theoretical') return theoreticalText || text;
                if (userPreferences.learningStyle === 'Practical') return practicalText || text;
                return text;
            };

            return { userPreferences, shouldShowContent, adaptText };
        }, [userPreferences]);

        if (loadingPreferences) {
            return <div>Loading personalized content...</div>;
        }

        // Error aane par hum content block nahi karenge, default show kar denge
        return <WrappedComponent {...props} personalizationUtils={personalizationUtils} />;
    };
};

export default withPersonalization;