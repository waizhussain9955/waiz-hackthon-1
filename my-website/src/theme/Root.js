import React from 'react';
import HumanoidChatbot from '@site/src/components/HumanoidChatbot';

export default function Root({ children }) {
    return (
        <>
            {children}
            <HumanoidChatbot />
        </>
    );
}
