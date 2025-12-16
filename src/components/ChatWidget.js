import React, { useState } from 'react';
import './ChatWidget.css'; // Import the associated CSS file

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
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
            <p>Hello! How can I assist you today?</p>
          </div>
          <div className="chat-input-area">
            <input type="text" placeholder="Type your message..." className="chat-input" />
            <button className="send-button">Send</button>
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