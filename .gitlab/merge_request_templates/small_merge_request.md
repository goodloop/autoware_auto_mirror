## Description
<!-- What is the high-level purpose of this merge request? Link to existing issue -->

## Notes for reviewer
<!-- Items in addition to the checklist below that the reviewer should pay special attention to -->

## Pre-review checklist for the author before submitting for review

Every developer is encouraged to be familiar with our [contributor guidelines](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html).

1. [ ] MR title and description help a friendly human understand the problem solved
1. [ ] Sensible notes for the reviewer added to the section above to facilitate review
1. [ ] MR fits the criteria of a "small change" listed below
1. [ ] "WIP" or "Draft" removed from the MR title
1. [ ] MR has a link to the original issue in the description, if it exists
1. [ ] If the source branch is on a fork, MR is configured to *allow commits from developers with access to push to the target branch*
1. [ ] Target branch set correctly. Default: `master`
1. [ ] MR assigned to a capable reviewer. Default: @JWhitleyWork

<details>
<summary markdown="span"><a name="general">What is meant by a "small change"?</a></summary>

This is a template with a trimmed-down checklist for small MRs. Use it when no new functions, classes or other things that require testing have been added.

Examples are pure documentation MRs, a fix for an off-by-one error, or changing log messages to be more informative. When in doubt, use the regular template.

</details>

## Checklist for the reviewer

**Only the reviewer is allowed to make changes in this section!**

Mark all the items that are done, and cross out items not applicable to this MR.

1. Basic checks
   1. [ ] The MR title describes what is being done on the ticket
   1. [ ] The MR does not introduce testable code
   1. [ ] The first commit has a proper [commit message](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/contributor-guidelines.html#contributors-guidelines-commits) to be used as a basis for the squashed commit created at the very end
1. Code correctness
   1. [ ] The problem/feature is solved (reproducibly)
   1. [ ] The solution is performant enough for the use case in mind
   1. [ ] Any disabled lints inside the code or at the package level are justified
1. Open work
   1. [ ] Any added source-code comment about future work refers to a follow-up GitLab issue explicitly; e.g., `// TODO #551 refactor code below`

If the MR provides an improvement, don't hesitate to add a :thumbsup: emoji for a neat line of code or a "Thanks for implementing this" comment. This will reward the MR author and prevent the review from being only about what still needs to be improved.

## Post-review checklist for the author

After receiving approval:

1. [ ] All checkboxes in the MR checklist are checked or crossed out. Syntax example: `1. [ ] ~~this item crossed out~~`
1. [ ] Assign MR to maintainer with sufficient rights to merge. Default: @JWhitleyWork
