import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

export default function HumanoidChatbot() {
    const [isOpen, setIsOpen] = useState(false);
    const [query, setQuery] = useState('');
    const [messages, setMessages] = useState([
        {
            role: 'assistant',
            content: (
                <div className={styles.welcomeContent}>
                    <p>👋 Hi! I'm your <strong>Physical AI & Humanoid Robotics</strong> assistant. I can help you with:</p>
                    <ul>
                        <li>ROS 2 and robotics middleware</li>
                        <li>Digital twin simulation</li>
                        <li>NVIDIA Isaac and navigation</li>
                        <li>Vision-Language-Action systems</li>
                    </ul>
                    <div className={styles.tipBox}>
                        💡 <strong>Tip:</strong> Select any text on the page, then ask me about it!
                    </div>
                </div>
            )
        }
    ]);
    const [selectedText, setSelectedText] = useState('');
    const [loading, setLoading] = useState(false);
    const messagesEndRef = useRef(null);

    const scrollToBottom = () => {
        messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
    };

    useEffect(() => {
        scrollToBottom();
    }, [messages]);

    useEffect(() => {
        const handleSelection = () => {
            const text = window.getSelection().toString();
            if (text) {
                setSelectedText(text);
            }
        };
        document.addEventListener('mouseup', handleSelection);
        return () => document.removeEventListener('mouseup', handleSelection);
    }, []);

    const handleSend = async () => {
        if (!query) return;
        setLoading(true);

        const newUserMsg = { role: 'user', content: query };
        const history = messages.map(m => ({
            role: m.role,
            content: typeof m.content === 'string' ? m.content : 'Welcome message'
        }));

        setMessages(prev => [...prev, newUserMsg]);
        setQuery('');

        try {
            const isLocal = window.location.hostname === "localhost" || window.location.hostname === "127.0.0.1";
            const API_URL = isLocal ? "http://localhost:8001/api/chat" : "/api/chat";
            const response = await fetch(API_URL, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    query: query,
                    selected_text: selectedText,
                    history: history.slice(-5)
                }),
            });

            if (!response.ok) {
                const errorData = await response.json().catch(() => ({}));
                throw new Error(errorData.detail || `Server returned ${response.status}`);
            }

            const data = await response.json();
            setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
            setSelectedText('');
        } catch (error) {
            console.error("Chat Error:", error);
            setMessages(prev => [...prev, { role: 'assistant', content: "Sorry, I'm having trouble connecting to my central brain. Please check your backend." }]);
        } finally {
            setLoading(false);
        }
    };

    const clearChat = () => {
        setMessages([messages[0]]);
    };

    return (
        <div className={styles.chatbotWrapper}>
            {!isOpen && (
                <button
                    className={styles.launcher}
                    onClick={() => setIsOpen(true)}
                >
                    <img src="https://img.icons8.com/ios-filled/50/ffffff/bot.png" alt="bot" />
                </button>
            )}

            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.header}>
                        <div className={styles.headerLeft}>
                            <div className={styles.botIcon}>🤖</div>
                            <span>Physical AI Assistant</span>
                        </div>
                        <div className={styles.headerRight}>
                            <button onClick={clearChat} title="Clear Chat" className={styles.headerBtn}>
                                🗑️
                            </button>
                            <button onClick={() => setIsOpen(false)} title="Close" className={styles.headerBtn}>
                                ✕
                            </button>
                        </div>
                    </div>

                    <div className={styles.messages}>
                        {messages.map((m, i) => (
                            <div key={i} className={m.role === 'user' ? styles.userMsg : styles.aiMsg}>
                                {m.content}
                            </div>
                        ))}
                        {loading && (
                            <div className={styles.aiMsg}>
                                <div className={styles.typingIndicator}>
                                    <span></span><span></span><span></span>
                                </div>
                            </div>
                        )}
                        <div ref={messagesEndRef} />
                    </div>

                    <div className={styles.footer}>
                        {selectedText && (
                            <div className={styles.selectionHint}>
                                <span>Selected: "{selectedText.substring(0, 20)}..."</span>
                                <button onClick={() => setSelectedText('')}>×</button>
                            </div>
                        )}
                        <div className={styles.inputArea}>
                            <input
                                value={query}
                                onChange={(e) => setQuery(e.target.value)}
                                onKeyPress={(e) => e.key === 'Enter' && handleSend()}
                                placeholder="Ask about robotics, ROS 2, or AI..."
                            />
                            <button onClick={handleSend} className={styles.sendBtn}>
                                ➤
                            </button>
                        </div>
                    </div>
                </div>
            )}
        </div>
    );
}
