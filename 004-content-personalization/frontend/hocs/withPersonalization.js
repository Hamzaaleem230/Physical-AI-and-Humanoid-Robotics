import React, { useState, useEffect, useMemo } from 'react';

// This HOC (Higher-Order Component) is designed to wrap content components
// and inject personalization logic based on user preferences.
const withPersonalization = (WrappedComponent) => {
    return function PersonalizableComponent(props) {
        const [userPreferences, setUserPreferences] = useState(null);
        const [loadingPreferences, setLoadingPreferences] = useState(true);
        const [preferencesError, setPreferencesError] = useState(null);

        useEffect(() => {
            const fetchUserPreferences = async () => {
                setLoadingPreferences(true);
                setPreferencesError(null);
                try {
                    // This is a placeholder for fetching user preferences from the backend.
                    // In a real application, you would make an API call here,
                    // likely including an Authorization header with a JWT token.
                    // Example:
                    // const response = await fetch('/personalization/preferences', {
                    //     headers: {
                    //         'Authorization': `Bearer ${YOUR_JWT_TOKEN}`
                    //     }
                    // });
                    // if (!response.ok) {
                    //     throw new Error(`Failed to fetch preferences: ${response.statusText}`);
                    // }
                    // const data = await response.json();
                    // setUserPreferences(data);

                    // Mocking user preferences for demonstration
                    await new Promise(resolve => setTimeout(resolve, 500));
                    setUserPreferences({
                        skillLevel: 'Beginner', // Can be 'Beginner', 'Intermediate', 'Expert'
                        interests: ['ROS2', 'Computer Vision'],
                        learningStyle: 'Practical', // Can be 'Theoretical', 'Practical', 'Balanced'
                    });
                } catch (error) {
                    console.error("Error fetching user preferences:", error);
                    setPreferencesError(error);
                } finally {
                    setLoadingPreferences(false);
                }
            };

            fetchUserPreferences();
        }, []); // Fetch preferences once on mount

        // Memoize personalization utilities to avoid re-creating them on every render
        const personalizationUtils = useMemo(() => {
            if (!userPreferences) return {};

            // Example utility: determine if a content section should be visible
            const shouldShowContent = (minSkillLevel) => {
                const skillLevelOrder = { 'Beginner': 0, 'Intermediate': 1, 'Expert': 2 };
                const userLevel = skillLevelOrder[userPreferences.skillLevel];
                const requiredLevel = skillLevelOrder[minSkillLevel];
                return userLevel >= requiredLevel;
            };

            // Example utility: adapt text based on learning style
            const adaptText = (text, theoreticalText, practicalText) => {
                if (userPreferences.learningStyle === 'Theoretical') {
                    return theoreticalText || text;
                } else if (userPreferences.learningStyle === 'Practical') {
                    return practicalText || text;
                }
                return text; // Default or 'Balanced'
            };

            return {
                userPreferences,
                shouldShowContent,
                adaptText,
                // Add more personalization utilities here as needed
            };
        }, [userPreferences]);

        if (loadingPreferences) {
            return <div>Loading personalized content...</div>;
        }

        if (preferencesError) {
            return <div>Error loading personalization: {preferencesError.message}. Content may not be personalized.</div>;
        }

        // Pass userPreferences and any personalization-related functions as props to the WrappedComponent
        return <WrappedComponent {...props} personalizationUtils={personalizationUtils} />;
    };
};

export default withPersonalization;
