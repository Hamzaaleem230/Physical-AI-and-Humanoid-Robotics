import { useEffect, useRef, useCallback } from 'react';
import useAuth from '@site/src/hooks/useAuth';

const PERSONALIZATION_API_BASE_URL = 'https://hamzasyed001122-content-personalization-backend.hf.space/personalization';

const useReadingHistoryTracker = (contentId) => {
    const { jwtToken, userId } = useAuth();
    const startTimeRef = useRef(Date.now());
    const intervalRef = useRef(null);

    const sendTrackingData = useCallback(async () => {
        // Bypass for testing if token is missing
        const tokenToUse = jwtToken || 'test-token-123';

        if (!contentId) return;

        const timeSpent = Math.floor((Date.now() - startTimeRef.current) / 1000);
        if (timeSpent <= 0) return;

        // Reset start time for next interval immediately
        startTimeRef.current = Date.now();

        try {
            await fetch(`${PERSONALIZATION_API_BASE_URL}/track`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                    'Authorization': `Bearer ${tokenToUse}`
                },
                body: JSON.stringify({
                    content_id: contentId,
                    engagement_score: timeSpent,
                }),
            });
            console.log(`✅ Tracked ${contentId}: ${timeSpent}s`);
        } catch (e) {
            console.error("Error tracking reading history:", e);
        }
    }, [contentId, jwtToken]);

    useEffect(() => {
        startTimeRef.current = Date.now();
        intervalRef.current = setInterval(sendTrackingData, 30 * 1000);

        const handleVisibilityChange = () => {
            if (document.visibilityState === 'hidden') {
                sendTrackingData();
            } else {
                startTimeRef.current = Date.now();
            }
        };

        document.addEventListener('visibilitychange', handleVisibilityChange);

        return () => {
            clearInterval(intervalRef.current);
            document.removeEventListener('visibilitychange', handleVisibilityChange);
            sendTrackingData();
        };
    }, [contentId, sendTrackingData]);

    return null;
};

export default useReadingHistoryTracker;