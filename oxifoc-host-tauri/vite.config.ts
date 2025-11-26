import { defineConfig } from 'vite'
import { fileURLToPath, URL } from 'node:url'
import vue from '@vitejs/plugin-vue'
import tailwindcss from '@tailwindcss/vite'
import { THEMES, DEFAULT_THEME } from './src/constants/themes'
import vueDevTools from 'vite-plugin-vue-devtools'

const host = process.env.TAURI_DEV_HOST
const isProduction = process.env.NODE_ENV === 'production'

export default defineConfig({
  plugins: [
    {
      name: 'theme-inject',
      transform(code: string, id: string): { code: string; map: null } | undefined {
        if (id.endsWith('style.css')) {
          const daisyuiThemes = [DEFAULT_THEME, THEMES.DARK].join(', ');
          const injectedCode = `@plugin "daisyui" {
  themes: ${daisyuiThemes};
}`;
          const modifiedCode = code + '\n' + injectedCode;
          return {
            code: modifiedCode,
            map: null,
          };
        }
        return undefined;
      },
      enforce: 'pre',
    },
    tailwindcss(),
    vue(),
    ...(!isProduction ? [vueDevTools()] : []),
  ],
  // Vite options tailored for Tauri development and only applied in `tauri dev` or `tauri build`
  //
  // 1. prevent vite from obscuring rust errors
  clearScreen: false,

  // 2. tauri expects a fixed port, fail if that port is not available
  server: {
    port: 1420,
    strictPort: true,
    host: host || false,
    hmr: host
      ? {
          protocol: 'ws',
          host,
          port: 1421,
        }
      : undefined,
    watch: {
      // 3. tell vite to ignore watching `src-tauri`
      ignored: ['**/src-tauri/**'],
    },
  },

  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
      // TimeChart GitHub version needs to be resolved from source since dist isn't pre-built
      'timechart': fileURLToPath(new URL('./node_modules/timechart/src/index.ts', import.meta.url)),
    },
  },

  build: {
    chunkSizeWarningLimit: 1000, // size in kB
  },

  base: './',
})
