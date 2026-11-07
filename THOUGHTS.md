1. For safety, never have velocity faster than stopping distance before next obstacle.
2. IMUs aren't reliable, but rocks screw with rangefinder. Use both for best results. SLAM? No optical flow though...
3. Low tolerance course means we can't hardcode. Use stacked goals instead, like go around wall to the right.
4. Fixed flight height for main journey. Midpoint of window.
5. Identifying the final target is going to be hard. Either have downwards facing camera or fixed delta offset.
