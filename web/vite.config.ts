import react from '@vitejs/plugin-react';
import { defineConfig } from 'vitest/config';

export default defineConfig({
    plugins: [react()],
    server: { port: 5173 },
    // NVIDIA's streaming library is most of the bundle (about 800 kB) and is needed at once.
    build: { chunkSizeWarningLimit: 1000 },
    test: {
        environment: 'node',
        include: ['src/**/*.test.ts'],
    },
});
