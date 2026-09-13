// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.
//
// Theme toggle for the generated docs. The stylesheet owns both palettes and switches on
// html[data-theme]; this only picks one.

(function () {
  "use strict";

  var storageKey = "vr-doxygen-theme";

  function systemPrefersDark() {
    return window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches;
  }

  function currentTheme() {
    var saved = window.localStorage ? window.localStorage.getItem(storageKey) : null;
    if (saved === "light" || saved === "dark") {
      return saved;
    }
    return systemPrefersDark() ? "dark" : "light";
  }

  function applyTheme(theme) {
    document.documentElement.setAttribute("data-theme", theme);
    var button = document.getElementById("vr-theme-toggle");
    if (button) {
      button.textContent = theme === "dark" ? "Light" : "Dark";
      button.setAttribute(
        "aria-label",
        "Switch to " + (theme === "dark" ? "light" : "dark") + " mode"
      );
    }
  }

  function init() {
    applyTheme(currentTheme());
    var button = document.getElementById("vr-theme-toggle");
    if (button) {
      button.addEventListener("click", function () {
        var next =
          document.documentElement.getAttribute("data-theme") === "dark" ? "light" : "dark";
        if (window.localStorage) {
          window.localStorage.setItem(storageKey, next);
        }
        applyTheme(next);
      });
    }
  }

  // Set before DOMContentLoaded, or dark-mode readers get a white flash on every page.
  applyTheme(currentTheme());

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
