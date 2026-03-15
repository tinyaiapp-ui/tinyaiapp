// Uređuj samo podatke ispod.
// Za svaki projekt možeš mijenjati naslov, opis, status, linkove i ostalo.
// Ako neki link nemaš, ostavi prazan string "".

window.APPS_DATA = [
  {
    title: "JobBoost Prompt Kit",
    category: "AI utility",
    status: "Released",
    version: "v1.0",
    platform: "PDF / Markdown",
    shortDescription: "120 ready-to-use AI prompts for job seekers — CV, cover letter, LinkedIn, interviews, and offer negotiation. Works with ChatGPT, Claude, Gemini, Copilot, and more.",
    progress: [
      "120 prompts across 6 job-search categories",
      "Blank document templates included (EU CV, US Resume, cover letter, interview prep)",
      "PDF-ready templates for direct AI upload",
      "Category cheat sheets with minimum required inputs per task"
    ],
    downloadUrl: "downloads/jobboost-prompt-kit.zip",
    demoUrl: "",
    githubUrl: "",
    updated: "2026-03-15"
  },
  {
    title: "Toranam Deploy Tool",
    category: "Desktop utility",
    status: "Testing",
    version: "v1.0",
    platform: "Windows / Linux / macOS (Python)",
    shortDescription: "Lightweight GUI tool for deploying files to a remote server via SSH — compare local and remote files, upload only selected or changed files, and manage systemd services in one click.",
    progress: [
      "SSH file comparison by size and modification time",
      "Upload selected files or only changed / new files",
      "One-click systemctl stop → upload → start workflow",
      "Saves connection profiles locally"
    ],
    downloadUrl: "",
    demoUrl: "",
    githubUrl: "",
    updated: "2026-03-15"
  },
  {
    title: "Robot Arm Controller",
    category: "Desktop utility",
    status: "In Development",
    version: "v0.5",
    platform: "Windows (Python + Arduino)",
    shortDescription: "Python desktop app for controlling a 6-axis robotic arm via Arduino — includes a sequence editor, 3D arm visualization, and full serial communication with onboard firmware.",
    progress: [
      "6-axis serial control via Arduino firmware",
      "Sequence editor with save / load / playback",
      "Real-time 3D arm visualization (matplotlib)",
      "Multi-language UI support"
    ],
    downloadUrl: "",
    demoUrl: "",
    githubUrl: "",
    updated: "2026-03-15"
  }
];
