import { useEffect, useRef, useCallback } from 'react';

/**
 * Custom React hook to track user reading history and send it to the backend.
 * @param {string} contentId A unique identifier for the content being viewed.
 * @param {string} jwtToken The JWT token for authentication.
 */
const useReadingHistoryTracker = (contentId, jwtToken) => {
    const startTimeRef = useRef(Date.now());
    const engagementScoreRef = useRef(0);
    const intervalRef = useRef(null);

    const sendTrackingData = useCallback(async () => {
        if (!contentId || !jwtToken) {
            // console.warn("Cannot track reading history: contentId or jwtToken is missing.");
            return;
        }

        const timeSpent = Math.floor((Date.now() - startTimeRef.current) / 1000);
        if (timeSpent <= 0) return; // Only send if some time has passed

        engagementScoreRef.current += timeSpent;
        startTimeRef.current = Date.now(); // Reset start time for the next interval

        // console.log(`Tracking: contentId=${contentId}, timeSpent=${timeSpent}, totalEngagement=${engagementScoreRef.current}`);

        try {
            // In a real app, you'd POST to your backend API
            // const response = await fetch('/personalization/track', {
            //     method: 'POST',
            //     headers: {
            //         'Content-Type': 'application/json',
            //         'Authorization': `Bearer ${jwtToken}`
            //     },
            //     body: JSON.stringify({
            //         contentId: contentId,
            //         engagementScore: timeSpent, // Send incremental score
            //     }),
            // });

            // if (!response.ok) {
            //     throw new Error(`HTTP error! status: ${response.status}`);
            // }
            // console.log("Reading history tracked successfully!");

            // Mocking API call for demonstration
            await new Promise(resolve => setTimeout(resolve, 300)); // Simulate network delay
            // console.log(`Mock: Tracked ${timeSpent}s for ${contentId}.`);

        } catch (e) {
            console.error("Error tracking reading history:", e);
        }
    }, [contentId, jwtToken]);

    useEffect(() => {
        // Start tracking when component mounts
        startTimeRef.current = Date.now();
        engagementScoreRef.current = 0;

        // Send tracking data periodically (e.g., every 30 seconds)
        intervalRef.current = setInterval(sendTrackingData, 30 * 1000); // 30 seconds

        const handleVisibilityChange = () => {
            if (document.visibilityState === 'hidden') {
                // If user leaves the page/tab, send remaining engagement data
                sendTrackingData();
            } else {
                // When user returns, reset start time to track new engagement
                startTimeRef.current = Date.now();
            }
        };

        document.addEventListener('visibilitychange', handleVisibilityChange);

        // Cleanup function: send final tracking data and clear interval when component unmounts
        return () => {
            clearInterval(intervalRef.current);
            document.removeEventListener('visibilitychange', handleVisibilityChange);
            sendTrackingData(); // Send any unsent data on unmount
        };
    }, [contentId, jwtToken, sendTrackingData]);

    // This hook doesn't return any values, its purpose is side-effects (tracking)
    return null;
};

export default useReadingHistoryTracker;
