---


---

<h2 id="python-coding">7. Python Coding</h2>
<p>This part covers how to write a publisher and subscriber node in python.</p>
<h3 id="writing-a-publisher">7.1 Writing a Publisher</h3>
<p>Change the directory to the package we created.</p>
<pre><code>$ roscd beginner_tutorials
</code></pre>
<p>Then create a ‘scripts’ folder to store out Python scriptes:</p>
<pre><code>$ mkdir scripts
$ cd scripts
</code></pre>
<p>Then download the example script <a href="https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py">talker.py</a> to your new scripts directory and <em>make it executable</em>:</p>
<pre><code>$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py
</code></pre>
<h3 id="explain-talker.py">7.2 Explain <code>talker.py</code></h3>
<p><em><strong>Part 1:</strong></em></p>
<pre><code>#!/usr/bin/env python
</code></pre>
<p>Every Python ROS node will have this declaration at the top. It makes sure your script is executable Python.</p>
<p><em><strong>Part 2:</strong></em></p>
<pre><code>import rospy
from std_msgs.msg import String
</code></pre>
<p>Import <code>rospy</code> for a ROS node program.  The <code>std_msgs.msg</code> import is so that we can reuse the <code>std_msgs/String</code> messgae type (simple string container) for publishing.</p>
<p><em><strong>Part 3:</strong></em></p>
<pre><code>    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
</code></pre>
<p>This section of code defines the talker’s interface to the rest of ROS.</p>
<ul>
<li>The <code>pub = rospy.Publisher("chatter", String, queue_size=10)</code> declares that your node is publishing to the chatter topic using the message type String. String here is actually the class <code>std_msgs.msg</code>. The <code>queue_size</code> argument limits the amount of queued messages if any subscriber is not receiving them fast enough. In older ROS distributions just omit the argument.</li>
<li>The next line, <code>rospy.init_node(NAME, ...)</code>, is very important as it tells rospy the name of your node – until rospy has this information, it cannot start communicating with the ROS <a href="http://wiki.ros.org/Master">Master</a>. In this case, your node will take on the name <code>talker</code>. <strong>NOTE</strong>: the name must be a <a href="http://wiki.ros.org/Names">base name</a>, i.e. it cannot contain any slashes “/”. <code>anonymous = True</code> ensures that your node has a unique name by adding random numbers to the end of NAME.</li>
</ul>
<p><em><strong>Part 4:</strong></em></p>
<pre><code>    rate = rospy.Rate(10) # 10hz
</code></pre>
<p>This line creates a Rate object <code>rate</code>. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!).</p>
<p><em><strong>Part 5:</strong></em></p>
<pre><code>    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
</code></pre>
<ul>
<li>This loop is a fairly standard rospy construct: checking the <code>rospy.is_shutdown()</code> flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise).</li>
<li>In this case, the “work” is a call to <code>pub.publish(hello_str)</code> that publishes a string to our <code>chatter</code> topic. The loop calls <code>rate.sleep()</code>, which sleeps just long enough to maintain the desired rate through the loop.</li>
<li>This loop also calls <code>rospy.loginfo(str)</code>, which performs triple-duty: the messages get printed to screen, it gets written to the Node’s log file, and it gets written to <a href="http://wiki.ros.org/rosout">rosout</a>. <a href="http://wiki.ros.org/rosout">rosout</a> is a handy for debugging: you can pull up messages using <a href="http://wiki.ros.org/rqt_console">rqt_console</a> instead of having to find the console window with your Node’s output.</li>
<li><code>std_msgs.msg.String</code> is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that <em>constructor args are in the same order as in the  .msg  file</em>. You can also pass in no arguments and initialize the fields directly, e.g.<pre><code>msg = String()
msg.data = str
# or
String(data=str)
</code></pre>
</li>
</ul>
<p><em><strong>Part 6:</strong></em></p>
<pre><code> try:
	 talker()
 except rospy.ROSInterruptException:
	 pass
</code></pre>
<p>In addition to the standard Python <code>__main__</code> check, this catches a <code>rospy</code>. <code>ROSInterruptException</code> exception, which can be thrown by <code>rospy.sleep()</code> and <code>rospy.Rate.sleep()</code> methods when Ctrl-C is pressed or your Node is otherwise shutdown. The reason this exception is raised is so that you don’t accidentally continue executing code after the <code>sleep()</code>.</p>
<h3 id="writing-a-subscriber">7.3 Writing a Subscriber</h3>
<p>Some same step to download a <a href="https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py">listener.py</a> file</p>
<pre><code>$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py
</code></pre>
<h3 id="explain-listener.py">7.4 Explain <code>listener.py</code></h3>
<p>The code of <code>listener.py</code> is simular to <code>talker.py</code>.</p>
<pre><code>	 rospy.init_node('listener', anonymous=True)
	 rospy.Subscriber("chatter", String, callback)
	 # spin() simply keeps python from exiting until this node is stopped
	 rospy.spin()
</code></pre>
<ul>
<li>This declares that your node subscribes to the <code>chatter</code> topic which is of type <code>std_msgs.msgs.String</code>. When new messages are received, <code>callback</code> is invoked with the<br>
message as the first argument.<pre><code>rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
</code></pre>
</li>
<li>The final addition,  <code>rospy.spin()</code>  simply keeps your node from exiting until the node has been shutdown. Unlike roscpp, <code>rospy.spin()</code> does not affect the subscriber callback functions, as those have their own threads.</li>
</ul>
<h3 id="building-your-nodes">7.5 Building your nodes</h3>
<p>This is to make sure that the autogenerated Python code for messages and services is created.<br>
Go to your catkin workspace and run <code>catkin_make</code>:</p>
<pre><code>$ cd ~/catkin_ws
$ catkin_make
</code></pre>
<p>Now that you have written a simple publisher and subscriber.</p>
<h2 id="examine-publisher-and-subscriber">8. Examine Publisher and Subscriber</h2>
<p>Always rememeber some works</p>
<pre><code>$ roscore
# In your catkin workspace
$ cd ~/catkin_ws
$ source ./devel/setup.*sh
</code></pre>
<p>Firstly, we can start to run <code>talker.py</code></p>
<pre><code>$ rosrun beginner_tutorials talker.py
</code></pre>
<p>Then, we can run the subscriber in another terminal:</p>
<pre><code>$ rosrun beginner_tutorials listener.py
</code></pre>
<h3 id="summary">9. Summary</h3>
<p>Until now, you are able to make nodes for a simple ROS simulation! Keep on studying for some interesting use!</p>

