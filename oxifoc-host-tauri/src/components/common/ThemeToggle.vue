<script setup lang="ts">
import { ref, onMounted } from 'vue'
import { THEMES, type ThemeName, DEFAULT_THEME, isValidTheme } from '../../constants/themes'

const storedTheme = localStorage.getItem('theme')
const theme = ref<ThemeName>(isValidTheme(storedTheme) ? storedTheme : DEFAULT_THEME)

const setTheme = (newTheme: ThemeName): void => {
  theme.value = newTheme
  localStorage.setItem('theme', newTheme)
  document.documentElement.setAttribute('data-theme', newTheme)
}

const toggleTheme = (): void => {
  const newTheme = theme.value === THEMES.LIGHT ? THEMES.DARK : THEMES.LIGHT
  setTheme(newTheme)
}

onMounted(() => {
  setTheme(theme.value)
})
</script>

<template>
  <label class="toggle text-base-content">
    <input
      type="checkbox"
      :checked="theme === THEMES.DARK"
      class="theme-controller"
      @change="toggleTheme" />
    <svg aria-label="sun" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
      <g
        stroke-linejoin="round"
        stroke-linecap="round"
        stroke-width="2"
        fill="none"
        stroke="currentColor">
        <circle cx="12" cy="12" r="4"></circle>
        <path d="M12 2v2"></path>
        <path d="M12 20v2"></path>
        <path d="m4.93 4.93 1.41 1.41"></path>
        <path d="m17.66 17.66 1.41 1.41"></path>
        <path d="M2 12h2"></path>
        <path d="M20 12h2"></path>
        <path d="m6.34 17.66-1.41 1.41"></path>
        <path d="m19.07 4.93-1.41 1.41"></path>
      </g>
    </svg>
    <svg aria-label="moon" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24">
      <g
        stroke-linejoin="round"
        stroke-linecap="round"
        stroke-width="2"
        fill="none"
        stroke="currentColor">
        <path d="M12 3a6 6 0 0 0 9 9 9 9 0 1 1-9-9Z"></path>
      </g>
    </svg>
  </label>
</template>
