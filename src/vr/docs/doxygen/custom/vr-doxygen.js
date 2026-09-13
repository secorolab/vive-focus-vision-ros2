// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Vamsi Kalagaturu
// See LICENSE for details.
//
// Theme toggle for the generated documentation, matching the mj_kdl_wrapper docs.
//
// The stylesheet defines both palettes and switches on html[data-theme]; this only decides which
// one is active. The choice is applied before DOMContentLoaded so a dark-mode reader does not get
// a white flash on every page load.
//
// The wrapper's version of this file also rewrites KDL and Python API references in code blocks
// into links. That is specific to a library whose API appears throughout its own examples, and is
// deliberately not carried over.

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

  // Before the DOM exists there is no button to update, but the attribute must already be set.
  applyTheme(currentTheme());

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
