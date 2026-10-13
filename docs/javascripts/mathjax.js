window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  }
};

document$.subscribe(() => {
  if (typeof MathJax !== "undefined" && typeof MathJax.typesetPromise === "function") {
    try {
      if (typeof MathJax.texReset === "function") MathJax.texReset();
      MathJax.typesetPromise();
    } catch (e) {
      // Suppress transient render errors
    }
  }
});
