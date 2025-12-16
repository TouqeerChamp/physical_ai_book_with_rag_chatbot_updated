import React, { useState } from 'react';
import './ChatWidget.css'; // Import the associated CSS file

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, text: 'Hello! How can I assist you today?', sender: 'bot' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async () => {
    if (input.trim() === '') return;

    const newMessage = { id: Date.now(), text: input, sender: 'user' };
    // Capture current input before setting it to empty
    const userQuery = input;

    setMessages(prev => [...prev, newMessage]);
    setInput('');
    setLoading(true);

    try {
        // --- NEW API CALL LOGIC TO BACKEND ---
        const response = await fetch('http://localhost:8000/api/query', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            // Use the captured userQuery
            body: JSON.stringify({ question: userQuery, top_k: 3 }),
        });

        const data = await response.json();
        const botResponse = { id: Date.now() + 1, text: data.answer || "Server responded, but no text was found.", sender: 'bot' };
        // --- END NEW API CALL LOGIC ---

        setMessages(prev => [...prev, botResponse]);
    } catch (error) {
        console.error('API Error:', error);
        const botResponse = {
            id: Date.now() + 1,
            text: "Error: Could not connect to the RAG API server (Is the server running on port 8000?). Please check your terminal.",
            sender: 'bot'
        };
        setMessages(prev => [...prev, botResponse]);
    } finally {
        setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      handleSendMessage();
    }
  };

  return (
    <div className="chat-widget">
      {isOpen ? (
        <div className="chat-window">
          <div className="chat-header">
            <h3>AI Assistant</h3>
            <button className="close-button" onClick={toggleChat}>×</button>
          </div>
          <div className="chat-messages">
            {messages.map((message) => (
              <div key={message.id} className={`message ${message.sender}`}>
                {message.text}
              </div>
            ))}
            {loading && <div className="message bot">Thinking...</div>}
          </div>
          <div className="chat-input-area">
            <input
              type="text"
              placeholder="Type your message..."
              className="chat-input"
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
            />
            <button
              className="send-button"
              onClick={handleSendMessage}
              disabled={loading}
            >
              Send
            </button>
          </div>
        </div>
      ) : (
        <button className="chat-toggle-button" onClick={toggleChat}>
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;