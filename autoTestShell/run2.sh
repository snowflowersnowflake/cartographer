tmux new -d -s save
tmux send-keys -t save.0 "script 1.txt" ENTER
tmux send-keys -t save.0 "echo helollll" ENTER
tmux send-keys -t save.0 "exit" ENTER

