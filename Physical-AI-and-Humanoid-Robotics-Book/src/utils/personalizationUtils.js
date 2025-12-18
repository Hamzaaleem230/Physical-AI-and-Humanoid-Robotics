// personalizationUtils.js
// This utility provides functions to enrich RAG chatbot prompts with user context.

/**
 * Enriches a user's chatbot prompt with their personalized context.
 * @param {string} userPrompt The original prompt from the user.
 * @param {object} userPreferences The user's preferences object, typically obtained from withPersonalization HOC.
 * @returns {string} The enriched prompt.
 */
export const enrichChatbotPrompt = (userPrompt, userPreferences) => {
    if (!userPreferences) {
        return userPrompt; // Return original prompt if no preferences are available
    }

    const { skillLevel, interests, learningStyle } = userPreferences;

    let context = `The user is a ${skillLevel} level learner. `;

    if (interests && interests.length > 0) {
        context += `Their interests include ${interests.join(', ')}. `;
    }

    context += `Their preferred learning style is ${learningStyle}. `;
    context += `Please tailor your response to this profile. `;

    return `${context}\n\nUser's question: ${userPrompt}`;
};

// You can add other personalization-related client-side utilities here.

