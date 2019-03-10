---


---

<h2 id="ros-nodes">5. ROS Nodes</h2>
<blockquote>
<p>Study with the official tutorials <a href="http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes">Understanding Nodes - ROS</a></p>
</blockquote>
<p>This part introduces ROS graph concepts and discusses the use of <code>roscore</code>, <code>rosnode</code>, and <code>rosrun</code> commandline tools.</p>
<h3 id="overview-of-graph-concepts">5.1 Overview of Graph Concepts</h3>
<ul>
<li><strong>Nodes:</strong> an executable that commutes with other nodes.</li>
<li><strong>Messages:</strong> ROS data type used when subscribing or publishing to a topic.</li>
<li><strong>Topics:</strong> publish messege to it or subsrcibe to it for messege.</li>
<li><strong>Master:</strong> Name service helps nodes find each other.</li>
<li><strong>rosout:</strong> ROS’s stdout/stderr.</li>
<li><strong>roscore:</strong> Master + rosout + parameter server.</li>
</ul>
<p>The concept of node in ROS is actually simple. A <em><strong>node</strong></em> really isn’t much more than an executable program within a ROS package.<br>
Nodes can publish or subcribe to a <em><strong>Topic</strong></em> for <em><strong>messages</strong></em> and also can provide or use a <em><strong>service</strong></em>.</p>
<h3 id="client-libraries">5.2 Client Libraries</h3>
<p>ROS client libraries allow nodes written in different languages:</p>
<ul>
<li>rospy = python client library</li>
<li>roscpp = C++ client library</li>
</ul>
<h3 id="command">5.3 Command</h3>
<p>This section talks about the <code>roscore</code>, <code>rosnode</code>, and <code>rosrun</code></p>
<p><strong>5.3.1 roscore</strong><br>
<code>roscore</code> is the first thing to run when using ROS.</p>
<pre><code>$ roscore
</code></pre>
<p>If you can see below info, then succeed</p>
<pre><code>...
started core service [/rosout]
</code></pre>
<p><strong>5.3.2 rosnode</strong><br>
Open up a new terminal, use<code>rosnode</code> to see what <code>roscore</code> did.<br>
<code>rosnode list</code> can return the existing nodes.</p>
<pre><code>$ rosnode list
	/rosout
</code></pre>
<blockquote>
<p><code>rosout</code> is always runing as it collects and logs the debugging output.</p>
</blockquote>
<p><code>rosnode info</code> command returns info about specific node. The result shows the <em>Publications</em>, <em>Subscriptions</em>, and <em>Services</em> of this node.</p>
<pre><code>$ rosnode info /rosout
</code></pre>
<p><strong>5.3.3 rosrun</strong><br>
<code>rosrun</code> allows us to use the package name to directly run a node within a package.</p>
<pre><code>$ rosrun [package_name] [node_name]
</code></pre>
<p>For example, open a new terminal:</p>
<pre><code>$ rosrun turtlesim turtlesim_node
</code></pre>
<p>Also, you can name the node when you run it!</p>
<pre><code>$ rosrun turtlesim turtlesim_node __name:=my_turtle
# Open another terminal
$ rosnode list
	/my_turtle
	/rosout
</code></pre>
<h2 id="ros-topics">6. ROS Topics</h2>
<h3 id="setup-simulation">6.1 Setup simulation</h3>
<p>Let 's setup the system if you didn’t follow the previous sections.</p>
<pre><code># Terminal 1: Initialization
$ roscore
# Terminal 2: The turtle window node
$ rosrun turtlesim turtlesim_node
# Terminal 3: The keyboard input node
$ rosrun turtlesim turtle_teleop_key
</code></pre>
<h3 id="concepts">6.2 Concepts</h3>
<p>The <code>turtlesim_node</code> and the <code>turtle_teleop_key</code> are communicating with each other over the ROS <em><strong>Topic</strong></em>.<br>
<code>turtle_teleop_key</code> is <em><strong>publishing</strong></em> the key strokes on a topic, while the <code>turtlesim_node</code> <em><strong>subscribes</strong></em> to the same topic to retrive key stokes.</p>
<h3 id="rqt_graph-tool">6.3 rqt_graph tool</h3>
<p><code>rqt_graph</code> enables us to see the visual graph description of nodes and topics.</p>
<pre><code>$ rosrun rqt_graph rqt_graph
</code></pre>
<h3 id="rostopic-tool">6.4 rostopic tool</h3>
<p>The <code>rostopic</code> tool allows you to get info about ROS topics.</p>
<pre><code># see what functions it has
$ rostopic -h
</code></pre>
<p><strong>rostopic list</strong><br>
<code>rostopic list</code> prints info about active topics</p>
<pre><code>$ rostopic list
</code></pre>
<p>There is suprise when add <code>-v</code> behind.</p>
<h3 id="ros-message">6.5 ROS Message</h3>
<p><strong>rostopic echo</strong><br>
<code>rostopic echo</code> continuously shows the <em><strong>message</strong></em> published on a topic.</p>
<pre><code>$ rostopic echo [topic]
</code></pre>
<p><strong>rostopic type</strong><br>
<code>rostopic type</code> returns the <em><strong>message</strong></em> type of any topic.</p>
<pre><code>$ rostopic type [topic]
</code></pre>
<p>For example, try</p>
<pre><code>$ rostopic type /turtle1/cmd_vel
	geometry_msgs/Twist
</code></pre>
<p>It seems like a standard message type of ROS, we can see the retails with <code>rosmsg</code></p>
<pre><code>$ rosmsg show geometry_msgs/Twist
	geometry_msgs/Vector3 linear
	  float64 x
	  float64 y
	  float64 z
	geometry_msgs/Vector3 angular
	  float64 x
	  float64 y
	  float64 z
</code></pre>
<h3 id="continued-rostopic">6.6 continued rostopic</h3>
<p>After knowing about ROS message, let us continue to see what other interesting things <code>rostopic</code> offers.</p>
<p><strong>rostopic pub</strong><br>
<code>rostopic pub</code> publishes message data onto a topic currently advertised.</p>
<pre><code>$ rostopic pub [topic] [msg_type] [args]
</code></pre>
<p>Example for turtlesim, try</p>
<pre><code>$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
</code></pre>
<p>The previous command will send a single message to turtlesim telling it to move with a linear velocity of 2.0, and an angular velocity of 1.8 .</p>
<ul>
<li>The option <code>-1</code> causes <code>rostopic</code> to only publish one message then exit.</li>
<li>The double-dash <code>--</code> tells that none of the following argu is an option. It helps when something the argument has negative numbers <code>-xx</code>, which cause conflict with options.</li>
<li>The message type decides that we need two of three floating point number. These arguments are actually in YAML syntax, which is described more in the <a href="http://wiki.ros.org/ROS/YAMLCommandLine">YAML command line documentation</a>.</li>
</ul>
<p>You may have noticed that the turtle has stopped moving; this is because the turtle requires a steady stream of commands at 1 Hz to keep moving. We can publish a steady stream of commands using <code>rostopic pub -r</code> command:</p>
<pre><code>$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
</code></pre>
<p>In this case you can see a new node <code>/rostopic_...</code>, which is generated by <code>rostopic pub -r</code>. The number right after <code>-r</code> is the frequency <em>[Hz]</em> of publishing.</p>
<p><strong>rostopic hz</strong><br>
<code>rostopic hz</code> reports the rate at which data is published.</p>
<pre><code>$ rostopic hz [topic]
</code></pre>
<p>Example, try</p>
<pre><code>$ rostopic hz /turtle1/pose
	subscribed to [/turtle1/pose]
	average rate: ....
</code></pre>
<p>Then we can know this topic is published at …Hz.</p>
<p><strong>rostopic type | rosmsg show</strong><br>
A conjuntion of this two command is a convenient tool to directly know a message content of a topic.</p>
<pre><code>$ rostopic type /turtle1/cmd_vel | rosmsg show
	geometry_msgs/Vector3 linear
	  float64 x
	  float64 y
	  float64 z
	geometry_msgs/Vector3 angular
	  float64 x
	  float64 y
	  float64 z
</code></pre>
<p>which is equal to <code>rostopic type &lt;topic&gt;</code> and input the return to <code>rosmsg show &lt;msg_type&gt;</code>.</p>
<h3 id="rqt_plot">6.7 rqt_plot</h3>
<p><code>rqt_plot</code> displays a scrolling time plot of the data published on topics.</p>
<pre><code>$ rosrun rqt_plot rqt_plot
</code></pre>
<p>search topic such as typing <code>/turtle1/pose/x</code>.<br>
Make sure you know the message content, for example, the <code>/turtle1/pose</code> is a type of turtlesim/Pose, which contains <em>x</em>, <em>y</em>, <em>theta</em>, etc.</p>

