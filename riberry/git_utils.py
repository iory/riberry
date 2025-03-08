import logging
import urllib

import git
import requests


def is_valid_github_url(url):
    try:
        response = requests.head(url, allow_redirects=True, timeout=5)
        return response.status_code == 200
    except requests.exceptions.RequestException as e:
        print(f"Error checking URL {url}: {e}")
        return False


def update_repository_with_safe_stash_apply(repo_path):
    try:
        repo = git.Repo(repo_path)
    except git.exc.InvalidGitRepositoryError:
        logging.error("The specified path is not a valid Git repository: %s", repo_path)
        return False
    stash_created = False
    try:
        # If local uncommitted changes are detected, stash them (including untracked files)
        if repo.is_dirty(untracked_files=True):
            logging.info("Local changes found, stashing them.")
            repo.git.stash('save', '--include-untracked')
            stash_created = True
    except Exception as e:
        logging.error("Failed to stash local changes: %s", e)
        return False

    try:
        pull_info = repo.remotes.origin.pull("main")
        logging.info("Pulled from origin main: %s", pull_info)
    except Exception as e:
        logging.error("Failed to pull from origin main: %s", e)
        return False

    # If a stash was created earlier, attempt to apply it safely
    if stash_created:
        try:
            # Attempt to apply the stash changes
            repo.git.stash('apply')
            logging.info("Stash applied successfully.")
        except Exception as e:
            logging.error("Failed to apply stash: %s", e)
            return False

        # Check for merge conflicts after applying the stash
        if repo.index.unmerged_blobs():
            logging.error("Merge conflicts detected after stash apply.")
            try:
                # Optionally, reset the working directory to HEAD to revert applied changes
                repo.git.reset('--hard', 'HEAD')
                logging.info("Repository reset to HEAD to revert conflicts.")
            except Exception as e:
                logging.error("Failed to reset repository: %s", e)
            return False

        try:
            # If the stash apply was successful and no conflicts are detected, drop the stash entry
            repo.git.stash('drop')
            logging.info("Stash dropped successfully.")
        except Exception as e:
            logging.error("Failed to drop stash: %s", e)
            return False

    return True


def generate_github_issue_url(repo_owner, repo_name, title, body):
    """Generate a GitHub issue URL with pre-filled title and body."""
    base_url = f"https://github.com/{repo_owner}/{repo_name}/issues/new"
    params = {
        "title": title,
        "body": body
    }
    query_string = urllib.parse.urlencode(params)
    return f"{base_url}?{query_string}"
