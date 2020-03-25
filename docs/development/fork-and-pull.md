How to develop in a fork {#develop-in-a-fork}
========================

# Using fork-and-pull {#fork-and-pull}

Autoware.Auto follows the fork-and-pull model of Git use.
This means that developers should make a fork of the Autoware.Auto repository, develop in branches on that fork, and then make merge requests (the GitLab term; pull requests in GitHub terminology) from that fork to the central Autoware.Auto repository.

The basic flow is quite simple.

1. Make a *fork* of the Autoware.Auto repository.
   The Autoware.Auto repository is known as the *upstream repository* relative to your fork.
   You will need a GitLab account for this, but a free account is sufficient.
   This fork will initially be stored on the GitLab servers only.

1. Clone your fork locally to create a *local copy*.

1. In your local copy, create a new branch to do your work in.
   Give the branch a descriptive name.
   GitLab uses the pattern `[issuenumber]-[issue-name-with-hyphens]`.
   For example, if you are doing work for issue #42, "Calculate the answer to the ultimate question", you could name your branch like so:

   ```shell
   $ git checkout -b 42-calculate-the-answer-to-the-ultimate-question
   ```

   This will create a new branch and put your local working copy into it.
   It is important to note that **this new branch is still only stored on your computer**.
   Before you can create a merge request, it must be pushed to the copy of *your fork* on the GitLab server.

1. Do your work in your local copy, in the new branch.

1. Commit your changes to the branch in your local copy.

1. Now that you have changes in your local copy, it is time to get them to the server, from where you can create a fork.
   From this point is where you need to be careful about where you run commands to ensure you do not accidentally pollute your history.
   If your history is not clean and up-to-date, your merge request will not be able to be merged.

   Begin by setting the upstream repository location in your local repository's settings.
   *You only need to do this the first time.*

   ```shell
   $ git remote add upstream https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
   ```

   Now that the upstream repository location is set, bring your branch up-to-date with the upstream repository's master branch.

   ```shell
   $ git checkout master
   $ git fetch
   $ git merge upstream/master
   ```

   The final command updates the `master` branch of your local copy to match the `master` branch of the Autoware.Auto repository.
   It is very important to use the `git merge` command to do this.
   **Do not use the `git rebase` command** to bring your local copy's `master` branch up-to-date.
   The reason for this is that [you should never rebase commits that are public](https://git-scm.com/book/en/v2/Git-Branching-Rebasing), and all the commits in the `master` branch are by their nature public.

   If the final command above does not happen smoothly, then your local copy of the `master` branch, or possibly the copy of the `master` branch in your fork on the server, has become polluted with extra commits.
   If you have placed a change in the `master` branch by mistake, create a new branch from `master`, then reset `master` to match the upstream.

   ```shell
   $ git checkout -b my-branch-for-some-new-change
   $ git checkout master
   $ git reset --hard upstream/master
   $ git push -f
   ```

1. The `master` branch in your local copy now matches the `master` branch in the Autoware.Auto repository.
   (Note that the `master` branch in your fork *on the GitLab server* does not, but this is not important, and easily rectified by pushing the local version to the server if you wish.)
   To ensure that your branch will apply cleanly to the `master` branch on the Autoware.Auto repository, which is a prerequisite for making a merge request, you need to bring the recent changes from the `master` branch into the branch containing your work.
   This is done using the [`git rebase` command](https://git-scm.com/book/en/v2/Git-Branching-Rebasing).
   You will be rebasing your changes onto the latest commit in the `master` branch.

   ```shell
   $ git checkout 42-calculate-the-answer-to-the-ultimate-question
   $ git rebase master
   ```

1. Finally, push your changes to the copy of *your fork* on the GitLab server.

   If this is the first time you are pushing this branch, you need to tell Git where the branch on your fork (not the Autoware.Auto repository) is.

   ```shell
   $ git push --set-upstream origin 42-calculate-the-answer-to-the-ultimate-question
   ```

   If you have pushed this branch before, then you do not need to set the destination branch.

   ```shell
   $ git push
   ```

You now have a copy of your branch with your proposed changes in *your fork* on the GitLab servers.
Next, go to the branch in your fork or to the Autoware.Auto repository in the GitLab website, and click the button to create a new merge request.
Create a merge request from your branch of your fork to the `master` branch of the Autoware.Auto repository.

When you need to add additional changes to the branch (for example, in response to a comment during code review), you will need to repeat the `git push` step above.
If there have been other merge requests merged into the `master` branch since you last pushed, you will also need to repeat the merging into `master` and rebasing steps.


# Working with unmerged merge requests {#using-unmerged-mrs}

There are times when you are working on something in your branch, and you find that you need to include some work that is not yet merged but is available in a separate merge request.
It is possible to temporarily include this work in your own branch while you develop, without polluting the history of changes that eventually make up your merge request.

Note that doing this places the following restrictions on your own merge request.

1. Your merge request cannot be reviewed until the other merge request(s) you depend on has been merged into `master` on the Autoware.Auto repository first.
   You should mark the issue that gave rise to your merge request as blocked by the relevant other issues.
   You can also mark your merge request as a work-in-progress (prepend "WIP: " to its title) until the other merge requests it depends on are merged.
   However this is generally not preferred if it is possible to begin the review work on your merge request before the other merge requests have been merged.

1. You must rebase and ensure a clean history, containing only your changes, prior to making a merge request.
   If you follow the instructions here to rebase your branch before making a merge request then this should not be a problem.

To include the changes from another merge request in your own branch, prior to them being merged into the `master` branch, you need to get that branch into your local copy and merge it into your branch.

1. In your local copy, fetch the latest from the upstream repository.

   ```shell
   $ git fetch
   ```

1. Change to your branch and merge in the changes from the branch for the merge request you wish to use.

   ```shell
   $ git merge upstream/41-calculate-the-ultimate-question
   ```

1. When your branch is ready to be merged into `master` and the branch you depend on has been merged, follow the [steps above](#fork-and-pull) for how to rebase your branch.


# When to do multiple merge requests

Sometimes while developing a new feature, you may find a bug that needs fixing.
There may be cases of this where you cannot continue development of your feature until the bug is fixed.

In this situation, the correct thing to do is to first file a bug report, then create a new merge request containing the fix.
However, the source of that merge request *should not* be your branch for your new feature.
You should create a new, separate branch for the bug fix and follow the standard process to create a merge request and get the bug fix merged into the `master` branch on the Autoware.Auto repository.

To use the bug fix in the branch for your new feature *prior* to it being merged into `master`, you
can follow the steps above for working with unmerged merge requests](#using-unmerged-mrs).
