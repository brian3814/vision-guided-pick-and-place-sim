import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';
import { App } from './App';
import { keepTypingOutOfTheStream } from './kit/keyboard';
import './styles.css';

keepTypingOutOfTheStream();

createRoot(document.getElementById('root')!).render(
    <StrictMode>
        <App />
    </StrictMode>,
);
