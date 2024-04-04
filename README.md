# Autonomus-Lifter-Robot-System

Here we will be uploading all the process for the Robotics Implementation project

Please crate a branch with the name of the functionality on which you will be working (ex. space-recognition)

Follow **Github Linux Tutorials** to connect your computer through SSH and be able to pull and push changes

[Generating a new SSH key and adding it to the ssh-agent](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent)

[Adding a new SSH key to your GitHub account](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account)

[Testing your SSH connection](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/testing-your-ssh-connection)

Then clone the repository from GitHub
```
git clone <ssh repository link>
```

![SSH Link](../docs/images/ssh_link.png raw=true)

I've experimented issues to connect SSH with Tec's network, so I recommend you to connect using a hotspot (For me it worked with the cellphone) just when you clone the repository and or push changes. If you mannage to solve this issue let us know how

You can pull, stage, commit and push changes from the **Source Control** tab on VSCode, but you can also use these commands

Update your local version with the version uploaded to GitHub (strongly recommended before pushing and creating branches)
```
git pull
```

Create the branch of the funcitonality you'll be working on
```
git branch <your new branch> <base branch (current branch is default)>
```

Go to your branch to start editting
```
git checkout <your branch>
```

Stage your changes for commit
```
#Stage changes on specific files
git add <changed file>
#Stage all changes
git add --all
```

Commit your changes (Commit message is mandatory)
```
#Commit staged changes on specific files
git commit <file> -m "Commit Message"
#Commit all staged changes
git commit --all -m "Commit Message"
```

Push changes
```
git push
```

When pushing changes **for the first time** on your new branch
```
git push --set-upstream origin <your branch>
```