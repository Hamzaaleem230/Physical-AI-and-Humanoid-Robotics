import React, { useState, useEffect } from 'react';

const skillLevels = ['Beginner', 'Intermediate', 'Expert'];
const learningStyles = ['Theoretical', 'Practical', 'Balanced'];

function PreferenceForm({ initialPreferences, onSave }) {
    // Ye hai aapki "Select..." wali default state
    const defaultState = {
        skill_level: '', 
        interests: [],
        learning_style: '',
    };

    const [preferences, setPreferences] = useState(defaultState);

    // Jab page load ho, to form ko khali (default) hi rehne dein
    // Agar aap chahte hain ke purani settings nazar AAYEIN, to niche wala useEffect use karein
    // Lekin aapne kaha default chahiye, isliye hum state ko reset hi rakhenge.
    useEffect(() => {
        if (initialPreferences && initialPreferences.skill_level) {
            // Agar aap chahte hain ke save ke baad reset ho, to isey khali chor dein
            // setPreferences(initialPreferences); // Is line ko comment rehne dein default ke liye
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

    const isFormIncomplete = !preferences.skill_level || !preferences.learning_style || preferences.interests.length === 0;

    return (
        <form onSubmit={(e) => { e.preventDefault(); onSave(preferences); }} 
              style={{ padding: '20px', border: '1px solid #ccc', borderRadius: '8px' }}>
            
            <div style={{ marginBottom: '15px' }}>
                <label><b>Skill Level:</b></label><br/>
                <select
                    name="skill_level"
                    value={preferences.skill_level}
                    onChange={handleChange}
                    style={{ width: '100%', padding: '8px' }}
                >
                    <option value="">Select Skill Level</option>
                    {skillLevels.map(level => <option key={level} value={level}>{level}</option>)}
                </select>
            </div>

            <div style={{ marginBottom: '15px' }}>
                <label><b>Interests:</b></label><br/>
                {['ROS2', 'Computer Vision', 'Control Systems', 'Robotics', 'AI', 'Machine Learning'].map(interest => (
                    <div key={interest}>
                        <input
                            type="checkbox"
                            name="interests"
                            value={interest}
                            checked={preferences.interests.includes(interest)}
                            onChange={handleChange}
                        />
                        <label style={{ marginLeft: '8px' }}>{interest}</label>
                    </div>
                ))}
            </div>

            <div style={{ marginBottom: '15px' }}>
                <label><b>Learning Style:</b></label><br/>
                <select
                    name="learning_style"
                    value={preferences.learning_style}
                    onChange={handleChange}
                    style={{ width: '100%', padding: '8px' }}
                >
                    <option value="">Select Learning Style</option>
                    {learningStyles.map(style => <option key={style} value={style}>{style}</option>)}
                </select>
            </div>

            <button 
                type="submit" 
                disabled={isFormIncomplete} 
                style={{ 
                    padding: '10px 20px', 
                    backgroundColor: isFormIncomplete ? '#ccc' : '#25c2a0', 
                    color: 'white', border: 'none', borderRadius: '5px',
                    cursor: isFormIncomplete ? 'not-allowed' : 'pointer'
                }}
            >
                Save Preferences
            </button>
        </form>
    );
}

export default PreferenceForm;