# Introduction and Motivation

With this project, we used Deep Learning to create an AI which could learn about and successfully navigate complex states. Additionally, we wanted to test whether competitive learning would affect the convergent fitness of the AIs, and whether strategies would be adopted to 'counter' other agents.



## The Game

The game is, in essentials, air hockey. There is a square field with two goals, separated down the middle. A ball  (circle) is dropped in the center of the field at the start of the game. Two AIs play against one another. Each AI controls a smaller striker (circle) which they can give direction and velocity to, in order to hit and bounce the ball into a goal. Once an AI scores a goal, the game is won. There are 3 AIs training against one another. Only two play the game at once. However, each AI will only train against one other AI. Specifically, AI #1 can only train from its matches against AI #2, although it will still play matches against AI #3, which it will not learn from. AI #2 will only learn from its matches against AI #3. AI #3 will only learn from its matches against AI #1.



## Technical Rules

1. AIs control one striker (circle), which bounce off the Ball and walls. They also bounce off the line dividing their half of the field as if it were a wall. 
2. Strikers' speed decay at a constant rate down to 0.
3. AIs can give strikers a velocity vector ranging in magnitude from 0 to a constant maximum. They cannot apply a velocity to their striker if it is moving.
4. The Ball will bounce off of strikers and walls.
5. The Ball's speed will decay at  a constant rate down to 0.
6. If the Ball hits a goal, the AI on the other side wins.



## Our Hypothesis

We hypothesize two possible results of the training:

`Hypothesis 1` states that our method of competitive learning converges to all AIs sharing the same behaviors. 

`Hypothesis 2` states that our method converges upon unique behaviors for each AI.

We suspect that `Hypothesis 1` will prove to be correct for most runs of the algorithm. However, we suspect that due to initial variance of parameters and temporary convergence to locally optimal strategies, some runs of this experiment may present a temporary expression of 'Rock Paper Scissors' like behavior. In other words, we expect that sometimes AIs may learn to be very good at beating the AI they train against, but not be necessarily good in general. However, we do expect that this 'countering' behavior to persist if AIs have a long enough opportunity to train. We expect that a more general adeptness at the game will win out over any specific strategies.



## Training and Evaluation

Each AI will train against one of the others. (AI #1 trains against AI #2 who trains against AI #1 who trains against AI #1) The reward function will simply be whether the AI wins or loses. This presents the issue that early generations may struggle to learn for a long time, due to the difficulty of scoring a goal on accident. Therefore, we may need a reward function which encourages the AIs to learn to correlate the position of the ball. To do this, we can implement a time limit on games. In the event that a game hits the limit, a different reward function can be invoked which rewards the Ball being closer to the opponent's goal, and punishes the Ball being closer to your own goal.

To evaluate the AIs, we need to test the AIs win rates. If the AI's win rates have no tendency towards being favored against the AI they trained against, then we accept `Hypothesis 1`. However, if the win rates are decidedly favored against the AIs they trained against, then we accept `Hypothesis 2`, but acknowledge that with more training or different learning strategies, AIs could still eventually converge upon a more general strategy.





