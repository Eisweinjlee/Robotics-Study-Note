---


---

<h1 id="section-8---motion-planning-i">Section 8 - Motion Planning I</h1>
<p>The simple <strong>Cartesian Control</strong> always results in a straight-line motion for the end effector, which is efficient but in some cases not workable. The most <strong>frequent problem</strong> is the obstacles avoiding, so the robot needs to plan a path.</p>
<h2 id="robot-configuration-space">8.1 Robot Configuration Space</h2>
<p>If there is a 2-link manipulator and a infinitesimal obstable on the surface. One can always draw a line in <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><msub><mi>q</mi><mn>1</mn></msub><mo>−</mo><msub><mi>q</mi><mn>2</mn></msub></mrow><annotation encoding="application/x-tex">q_1-q_2</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.77777em; vertical-align: -0.19444em;"></span><span class="mord"><span class="mord mathit" style="margin-right: 0.03588em;">q</span><span class="msupsub"><span class="vlist-t vlist-t2"><span class="vlist-r"><span class="vlist" style="height: 0.301108em;"><span class="" style="top: -2.55em; margin-left: -0.03588em; margin-right: 0.05em;"><span class="pstrut" style="height: 2.7em;"></span><span class="sizing reset-size6 size3 mtight"><span class="mord mtight">1</span></span></span></span><span class="vlist-s">​</span></span><span class="vlist-r"><span class="vlist" style="height: 0.15em;"><span class=""></span></span></span></span></span></span><span class="mspace" style="margin-right: 0.222222em;"></span><span class="mbin">−</span><span class="mspace" style="margin-right: 0.222222em;"></span></span><span class="base"><span class="strut" style="height: 0.625em; vertical-align: -0.19444em;"></span><span class="mord"><span class="mord mathit" style="margin-right: 0.03588em;">q</span><span class="msupsub"><span class="vlist-t vlist-t2"><span class="vlist-r"><span class="vlist" style="height: 0.301108em;"><span class="" style="top: -2.55em; margin-left: -0.03588em; margin-right: 0.05em;"><span class="pstrut" style="height: 2.7em;"></span><span class="sizing reset-size6 size3 mtight"><span class="mord mtight">2</span></span></span></span><span class="vlist-s">​</span></span><span class="vlist-r"><span class="vlist" style="height: 0.15em;"><span class=""></span></span></span></span></span></span></span></span></span></span> planar to illustrate all the illegal configurations. If the obstable on the surface is a range, one can find an illegal range of <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><msub><mi>q</mi><mn>1</mn></msub><mo>−</mo><msub><mi>q</mi><mn>2</mn></msub></mrow><annotation encoding="application/x-tex">q_1-q_2</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.77777em; vertical-align: -0.19444em;"></span><span class="mord"><span class="mord mathit" style="margin-right: 0.03588em;">q</span><span class="msupsub"><span class="vlist-t vlist-t2"><span class="vlist-r"><span class="vlist" style="height: 0.301108em;"><span class="" style="top: -2.55em; margin-left: -0.03588em; margin-right: 0.05em;"><span class="pstrut" style="height: 2.7em;"></span><span class="sizing reset-size6 size3 mtight"><span class="mord mtight">1</span></span></span></span><span class="vlist-s">​</span></span><span class="vlist-r"><span class="vlist" style="height: 0.15em;"><span class=""></span></span></span></span></span></span><span class="mspace" style="margin-right: 0.222222em;"></span><span class="mbin">−</span><span class="mspace" style="margin-right: 0.222222em;"></span></span><span class="base"><span class="strut" style="height: 0.625em; vertical-align: -0.19444em;"></span><span class="mord"><span class="mord mathit" style="margin-right: 0.03588em;">q</span><span class="msupsub"><span class="vlist-t vlist-t2"><span class="vlist-r"><span class="vlist" style="height: 0.301108em;"><span class="" style="top: -2.55em; margin-left: -0.03588em; margin-right: 0.05em;"><span class="pstrut" style="height: 2.7em;"></span><span class="sizing reset-size6 size3 mtight"><span class="mord mtight">2</span></span></span></span><span class="vlist-s">​</span></span><span class="vlist-r"><span class="vlist" style="height: 0.15em;"><span class=""></span></span></span></span></span></span></span></span></span></span>, which becomes a surface.<br>
This space telling the configuration of robot is also called <em><strong>Robot Configuration Space (C-Space)</strong></em> . A visual illustration can be found in website <a href="https://robotics.cs.unc.edu/education/c-space/">Configuration Space Visualization</a>.</p>
<p>However, robots are higher dimension than 2 links, the C-space are relatively higher dimension. Although the space cannot be render for complex robot, it still exists and the same algorithms are used.</p>
<h2 id="robot-arms-vs-mobile-robots">8.2 Robot Arms vs Mobile Robots</h2>
<p>Motion Planning in robot arms and mobile robots can be very different, so that in other systems the motion palning problems can be distinct.</p>
<p><strong>Arms:</strong></p>
<ul>
<li><strong>High-dimensional</strong> C-space (often 6 dimensional)</li>
<li><strong>Discretizing map</strong> into grid is not tractable</li>
<li><strong>Polygonal C-space</strong> obstacle map is hard to compute</li>
<li>Knowing “where” you are on the map is generally easy</li>
</ul>
<p><strong>Mobile Robots:</strong></p>
<ul>
<li>Low-dimensional C-space (2 dimensional)</li>
<li>Discretizing map into grid is often done</li>
<li>Polygonal obstacle map can be available (e.g. floor plan)</li>
<li><strong>Knowing “where” you are</strong> on the map is generally hard</li>
</ul>
<h2 id="robot-arms-motion-planning">8.3 Robot Arms Motion Planning</h2>
<p>Assume we have models of obstacles in task space (Cartesian):</p>
<ul>
<li>Mapping obstacles from <em>task space</em> <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>X</mi></mrow><annotation encoding="application/x-tex">X</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.68333em; vertical-align: 0em;"></span><span class="mord mathit" style="margin-right: 0.07847em;">X</span></span></span></span></span> into <em>joint space</em> <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>Q</mi></mrow><annotation encoding="application/x-tex">Q</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.87777em; vertical-align: -0.19444em;"></span><span class="mord mathit">Q</span></span></span></span></span> is hard</li>
<li>But answering point queries (is this pose in collision?) is easy<br>
How can we do with this?</li>
</ul>
<p>This kind of method is called <em><strong>sampling-based motion planning algorithms</strong></em> or <em><strong>stochastic motion planning</strong></em>. The main idea is to take as many as random explorations on configuration space, and with the return information the validity is checked and steps can be done.</p>
<h3 id="rapidly-exploring-random-trees-rrt">8.3.1 Rapidly-Exploring Random Trees (RRT)</h3>
<p><strong>Input:</strong> start and goal point in C-Space<br>
<strong>Output:</strong> path from start to goal<br>
<strong>Algorithm:</strong></p>
<ul>
<li>Insert start point in tree</li>
<li>While tree cannot connect to goal directly:
<ul>
<li>Sample <strong>random point</strong> <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>r</mi></mrow><annotation encoding="application/x-tex">r</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.43056em; vertical-align: 0em;"></span><span class="mord mathit" style="margin-right: 0.02778em;">r</span></span></span></span></span> in C-space</li>
<li>Find point <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>p</mi></mrow><annotation encoding="application/x-tex">p</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.625em; vertical-align: -0.19444em;"></span><span class="mord mathit">p</span></span></span></span></span> in tree that is closest to <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>r</mi></mrow><annotation encoding="application/x-tex">r</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.43056em; vertical-align: 0em;"></span><span class="mord mathit" style="margin-right: 0.02778em;">r</span></span></span></span></span></li>
<li>Add branch of predefined length from <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>p</mi></mrow><annotation encoding="application/x-tex">p</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.625em; vertical-align: -0.19444em;"></span><span class="mord mathit">p</span></span></span></span></span> in direction of <span class="katex--inline"><span class="katex"><span class="katex-mathml"><math><semantics><mrow><mi>r</mi></mrow><annotation encoding="application/x-tex">r</annotation></semantics></math></span><span class="katex-html" aria-hidden="true"><span class="base"><span class="strut" style="height: 0.43056em; vertical-align: 0em;"></span><span class="mord mathit" style="margin-right: 0.02778em;">r</span></span></span></span></span></li>
<li>if new branch <strong>intersects obstacle</strong>:
<ul>
<li>discard new branch (or shorten)</li>
</ul>
</li>
</ul>
</li>
<li>Compute path from start to goal through tree</li>
<li>Shortcut path: for any two points in path, add direct line unless direct line <strong>intersects an obstacle</strong>.</li>
</ul>
<p><strong>Note:</strong> It is a simple explanation and there are many possible variations.	 for production-level algorithms.</p>
<p><strong>Extenal call</strong> - <strong>random point</strong>: call a random function, easy for high-level program language.<br>
<strong>Extenal call</strong> - <strong>intersects obstacle</strong>: it is done by discretizing line, and then checking individual points for collision.</p>
<h3 id="probabilistic-roadmaps-prm">8.3.2 Probabilistic Roadmaps (PRM)</h3>
<p><strong>Roadmap Construction:</strong></p>
<ul>
<li>While number of points in roadmap lower than threshold:
<ul>
<li>sample <strong>random</strong> point in C-space</li>
<li>if new point is not in collision:
<ul>
<li>connect new point to all other points in the roadmap, as long as lines do not <strong>intersect obstacles</strong>.</li>
</ul>
</li>
</ul>
</li>
</ul>
<p><strong>Input:</strong> start and goal point in C-space<br>
<strong>Output:</strong> path from start to goal<br>
<strong>Path finding:</strong></p>
<ul>
<li>connect start point to nearest point in roadmap such that connecting line does not <strong>intersect obstacle</strong>.</li>
<li>connect goal point to nearest point in roadmap such that connecting line does not <strong>intersect obstacle</strong>.</li>
<li><strong>find a path between start and goal going exclusively on the roadmap</strong></li>
</ul>
<h3 id="comments-of-sampling-based-method">8.3.3 Comments of Sampling-based Method</h3>
<ul>
<li><strong>Merit:</strong> only requires the ability to quickly check if a point in C-space is “legal” or not (often that means collision-free)</li>
<li>Many versions are <strong>probabilisticaly complete</strong>: if a path exists, it will be found in finite time.</li>
<li>In practice, these algorithms tend to be very effective in <strong>high-dimensional</strong> spaces.</li>
<li><strong>Demerit:</strong> There are also no guarantees regarding quality of solution:
<ul>
<li>not guaranteed to be the shortest</li>
<li>often needs post-processing (e.g. to eliminate zigs and zags)</li>
</ul>
</li>
</ul>
<h2 id="software-package---moveit">8.4 Software Package - MoveIt!</h2>
<p>Study this! <a href="http://moveit.ros.org/">MoveIt on ROS</a></p>

