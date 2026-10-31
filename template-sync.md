# Git Repository Duplication & Template Syncing Guide

This guide explains how to duplicate a template repository **while keeping its full commit history**, how to pull future updates from the template.

> Credit: [git-repo-template-duplication-guide | byAnushkai1 ](https://github.com/Anushkai1/git-repo-template-duplication-guide)

> Use this repo as its base: [Solvers-Quickstart](https://github.com/FTC-23511/SolversLib-Quickstart)
---

## How This Works

When you click GitHub's default "Use this template" button, GitHub takes a snapshot of the files, flattens the history, and gives you a single **"Initial Commit"**. It completely cuts ties with the template's past history.

To keep the history and receive future updates, we bypass GitHub's template button and use standard Git features:
1. **Cloning** copies the entire `.git` database directory, which contains every commit ever made.
2. **Remotes** are just bookmarks (URLs) pointing to different hosting locations on GitHub. A single local project folder can track multiple remotes at the same time:
    * `origin`: Points to your **new** project repository (where you push your work).
    * `template`: Points back to the **original template** repository (where you fetch updates).

> ⚠️ **Important Multi-Computer Note:** Git remotes are stored **locally** in your computer's `.git/config` file. They do not sync to GitHub. If you clone your new repository onto a second computer, it will only know about `origin`. You must re-register the `template` remote on that second computer to fetch template updates there.

---

## 1. Create a New Repository (Preserving Full History)

### Step A: Create an Empty Repo on GitHub
1. Go to GitHub and create a new repository.
2. **Do NOT** initialize it with a README, `.gitignore`, or a license. Keep it completely empty.
3. Copy the repository URL (e.g., `https://github.com/YOUR_USERNAME/YOUR_NEW_REPO.git`).

### Step B: Clone and Re-Remote via CLI
Open your terminal and run the following commands sequentially:

1. Clone the template repository into a new folder name
```bash
git clone https://github.com/YOUR_USERNAME/YOUR_TEMPLATE_REPO.git my-new-project
```

2. Navigate into your new project directory
```
cd my-new-project
```

3. Change 'origin' to point to your new empty GitHub repository
```
git remote set-url origin https://github.com/YOUR_USERNAME/YOUR_NEW_REPO.git
```

4. Verify that 'origin' points to your new repository, NOT the template
```
 git remote -v
```

5. Push the code along with its entire history to your new repo
```
git push -u origin main
```

---

## 2. Connect the Template & Sync Initial Updates

Once your new repository is created, you need to configure your local machine so it can talk to the template for future updates.

1. Register the original template as a secondary remote named 'template'
```
git remote add template https://github.com/YOUR_USERNAME/YOUR_TEMPLATE_REPO.git
```

2. Fetch the latest branches and commits from the template
```
git fetch template
```

3. Merge the template's main branch into your current local branch
```
git merge template/main
```
4. Push the synchronized code up to your new GitHub repository
```bash
git push origin main
```

---

## 3. Pulling Future Template Updates (Routine Workflow)

Whenever you make changes to the master template in the future and want to bring those updates into an existing project, run these three quick commands on your local machine:

1. Download new changes from the template repo
```
git fetch template
```

2. Combine the template changes into your active branch
```
git merge template/main
```

3. Upload the merged updates to your project's GitHub repo
```
git push origin main
```

---

## 4. Multi-Computer Setup (Working on a New PC)

If you clone your new project onto a **different computer**, Git will only remember `origin`. It will not know the template exists. To pull template updates on a second computer, you must register the remote again.

### Steps for the Second Computer:
1. Clone your project normally onto the new machine
```
git clone https://github.com/YOUR_USERNAME/YOUR_NEW_REPO.git
```

```
cd YOUR_NEW_REPO
```

2. Re-add the template remote link (Crucial Step!)

```
git remote add template https://github.com/YOUR_USERNAME/YOUR_TEMPLATE_REPO.git
```

3. Test the connection by fetching
```
git fetch template
```

Once these steps are run on the new computer, you can use the routine workflow from **Section 3** freely on this machine.

---

## Limitations, Issues, & Best Practices

### 1. Merge Conflicts
If you modify a file (e.g., `src/theme/global.css` or `package.json`) in your new project, and you later modify that exact same file inside the template repo, Git will throw a **Merge Conflict** when you try to sync.
* **Solution:** Keep template code as modular and generic as possible. If a conflict occurs, open your code editor, choose which lines to keep, stage the file with `git add .`, and commit to finish the merge.

### 2. Hardcoded Template Identities
Because you copied the entire history, configuration files like `package.json`, application IDs, app names, or `.env.example` configurations will match the template exactly.
* **Solution:** Change the `"name"` and `"version"` properties in your `package.json` immediately after completing Section 1.

### 3. Git Tags and Releases
A standard clone brings along any Git release tags (e.g., `v1.0.0`) from the template. This can trigger unintended automation if your new repository utilizes CI/CD deployment pipelines based on tags.
* **Solution:** Clear or manage tags in your new repository if your deployment tools rely heavily on automated version tags.