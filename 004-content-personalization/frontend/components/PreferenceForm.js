import React, { useState, useEffect } from 'react';

const skillLevels = ['Beginner', 'Intermediate', 'Expert'];
const learningStyles = ['Theoretical', 'Practical', 'Balanced'];

function PreferenceForm({ initialPreferences, onSave }) {
    const [preferences, setPreferences] = useState(initialPreferences || {
        skillLevel: 'Beginner',
        interests: [],
        learningStyle: 'Theoretical',
    });

    useEffect(() => {
        if (initialPreferences) {
            setPreferences(initialPreferences);
        }
    }, [initialPreferences]);

    const handleChange = (e) => {
        const { name, value, type, checked } = e.target;
        if (name === 'interests') {
            setPreferences(prev => {
                const newInterests = checked
                    ? [...prev.interests, value]
                    : prev.interests.filter(interest => interest !== value);
                return { ...prev, interests: newInterests };
            });
        } else {
            setPreferences(prev => ({ ...prev, [name]: value }));
        }
    };

    const handleSubmit = (e) => {
        e.preventDefault();
        onSave(preferences);
    };

    return (
        <form onSubmit={handleSubmit} className="preference-form">
            <h2>Your Learning Preferences</h2>

            <div className="form-group">
                <label htmlFor="skillLevel">Skill Level:</label>
                <select
                    id="skillLevel"
                    name="skillLevel"
                    value={preferences.skillLevel}
                    onChange={handleChange}
                    required
                >
                    {skillLevels.map(level => (
                        <option key={level} value={level}>{level}</option>
                    ))}
                </select>
            </div>

            <div className="form-group">
                <label>Interests (select all that apply):</label>
                {['ROS2', 'Computer Vision', 'Control Systems', 'Robotics', 'AI', 'Machine Learning'].map(interest => (
                    <div key={interest}>
                        <input
                            type="checkbox"
                            id={interest}
                            name="interests"
                            value={interest}
                            checked={preferences.interests.includes(interest)}
                            onChange={handleChange}
                        />
                        <label htmlFor={interest}>{interest}</label>
                    </div>
                ))}
            </div>

            <div className="form-group">
                <label htmlFor="learningStyle">Learning Style:</label>
                <select
                    id="learningStyle"
                    name="learningStyle"
                    value={preferences.learningStyle}
                    onChange={handleChange}
                    required
                >
                    {learningStyles.map(style => (
                        <option key={style} value={style}>{style}</option>
                    ))}
                </select>
            </div>

            <button type="submit">Save Preferences</button>
        </form>
    );
}

export default PreferenceForm;
