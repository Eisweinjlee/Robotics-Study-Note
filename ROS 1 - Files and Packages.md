---


---

<h1 id="ros-tutorials">ROS Tutorials</h1>
<blockquote>
<p>study from the ROS tutorials beginner level.<br>
skip the steps to install ROS.</p>
</blockquote>
<h2 id="create-a-ros-workspace">1. Create a ROS workspace</h2>
<pre><code>$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
</code></pre>
<p>The <a href="http://wiki.ros.org/catkin/commands/catkin_make">catkin_make</a> command is a convenience tool for working with <a href="http://wiki.ros.org/catkin/workspaces">catkin workspaces</a>. Running it the first time in your workspace, it will create a CMakeLists.txt link in your ‘src’ folder. Additionally, if you look in your current directory you should now have a ‘build’ and ‘devel’ folder.</p>
<pre><code>$ source devel/setup.*sh
</code></pre>
<p>To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you’re in.</p>
<pre><code>$ echo $ROS_PACKAGE_PATH
</code></pre>
<h2 id="ros-filesystem">2. ROS Filesystem</h2>
<p>This part we will inspect a package in ros-tutorials</p>
<pre><code>$ sudo apt-get install ros-&lt;distro&gt;-ros-tutorials
</code></pre>
<h3 id="concepts">2.1 Concepts</h3>
<ul>
<li><strong>Packages:</strong> software organization unit of ROS code. Each package contain executables, scriptes, or other artifacts.</li>
<li><strong>Manifests:</strong> a manifest is a description of a package, about its version, maintainer, license, etc.</li>
</ul>
<h3 id="filesystem-tools">2.2 Filesystem Tools</h3>
<ol>
<li><em><strong>rospack find</strong></em>: returns the path to package.</li>
</ol>
<pre><code>$ rospack find [package_name]
</code></pre>
<ol start="2">
<li><em><strong>roscd</strong></em>: change dir directly to a package or a stack.</li>
</ol>
<pre><code>$ $ roscd [locationname[/subdir]]
</code></pre>
<ul>
<li>Examples of roscd:</li>
</ul>
<pre><code>$ roscd roscpp	(package)
$ roscd roscpp/cmake	(subdirectories)
</code></pre>
<ul>
<li><em>roscd log</em>: take you to the folder where ROS stores log files.</li>
</ul>
<ol start="3">
<li><em><strong>rosls</strong></em>: the rosbash’s ls function, return the content of package or subdirectories.</li>
</ol>
<pre><code>$ rosls [locationname[/subdir]]
</code></pre>
<h2 id="create-a-package">3. Create a Package</h2>
<h3 id="requirements-of-a-package">3.1 Requirements of a Package</h3>
<ul>
<li>a catkin compliant <em>package.xml</em> file (contains meta info about the package)</li>
<li>a <em>CMakeLists.txt</em> file for catkin.</li>
<li>package’s own folder</li>
</ul>
<p>Therefore, the simplest package structure should be:</p>
<pre><code>my_package/
  CMakeLists.txt
  package.xml
</code></pre>
<h3 id="packages-in-a-catkin-workspace">3.2 Packages in a Catkin Workspace</h3>
<p>Packages are stored in the <code>/src</code> and a trival workspace might look like:</p>
<pre><code>workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
</code></pre>
<h3 id="create-a-catkin-package">3.3 Create a Catkin Package</h3>
<p>We have  already created a workspace in <em>1. Create a ROS workspace</em> above.</p>
<pre><code>$ cd ~/catkin_ws/src
</code></pre>
<p>Now use the <code>catkin_create_pkg</code> script to create a new package called ‘beginner_tutorials’ which depends on std_msgs, roscpp, and rospy:</p>
<pre><code>$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
</code></pre>
<p>This will create a <code>beginner_tutorials</code> folder which contains a <em>package.xml</em> and a <em>CMakeLists.txt</em>.</p>
<p><em><strong>catkin_create_pkg</strong></em></p>
<pre><code>$ catkin_create_pkg &lt;package_name&gt; [depend1] [depend2] [depend3]
</code></pre>
<h3 id="build-workspace-and-sourcing-setup-file">3.4 Build Workspace and Sourcing setup file</h3>
<p>To build those news packages into the catkin workspace:</p>
<pre><code>$ cd ~/catkin_ws
$ catkin_make
</code></pre>
<p>After the workspace has been built it has created a workplace structure and we can see the devel.<br>
To add the workspace to your ROS environment you need to source the generated setup file:</p>
<pre><code>$ . ~/catkin_ws/devel/setup.bash
</code></pre>
<h3 id="package-dependencies">3.5 Package Dependencies</h3>
<p>After <em>catkin_create_pkg</em>, we can view the <strong>first-order dependencies</strong> with those <em>rospack</em> tool.</p>
<pre><code>$ rospack depends1 beginner_tutorials
	roscpp
	rospy
	std_msgs
</code></pre>
<p>These dependencies for a package are stored in the <strong>package.xml</strong> file:</p>
<pre><code>$ roscd beginner_tutorials
$ cat package.xml
	  ...
	  &lt;build_depend&gt;.....&lt;/build_depend&gt;
	  ...
</code></pre>
<p>Except for the first-order dependencies, the <strong>indirect dependencies</strong> are also be able to check.</p>
<pre><code>$ rospack depends1 rospy
</code></pre>
<p>or show all level dependencies with <em>rospack depends</em> (without 1).</p>
<pre><code>$ rospack depends beginner_tutorials
</code></pre>
<h3 id="learn-and-customize-package">3.6 Learn and Customize Package</h3>
<p><strong>Package.xml</strong><br>
The generated <a href="http://wiki.ros.org/catkin/package.xml">package.xml</a> should be in your new package.</p>
<p>It contains following parts:</p>
<ul>
<li><strong>Description tag</strong>: can be anything describing the package.</li>
</ul>
<pre><code>&lt;description&gt;The beginner_tutorials package&lt;/description&gt;
</code></pre>
<ul>
<li><strong>Maintainer tags</strong>: maintainer’s info.</li>
</ul>
<pre><code>&lt;maintainer email="you@yourdomain.tld"&gt;Your Name&lt;/maintainer&gt;
</code></pre>
<ul>
<li><strong>License tags</strong>: choose one open source license, BSD is recommended.</li>
</ul>
<pre><code>&lt;license&gt;BSD&lt;/license&gt;
</code></pre>
<ul>
<li><strong>Dependencies tags</strong>: dependencies split into <code>buildtool_depend</code>, <code>build_depend</code> and <code>exec_depend</code> three types.</li>
</ul>
<pre><code>&lt;buildtool_depend&gt;catkin&lt;/buildtool_depend&gt;
&lt;build_depend&gt;roscpp&lt;/build_depend&gt;
&lt;exec_depend&gt;roscpp&lt;/exec_depend&gt;
</code></pre>
<p><strong>CMakeLists.txt</strong><br>
The  <a href="http://wiki.ros.org/catkin/CMakeLists.txt">CMakeLists.txt</a>  file created by  <em>catkin_create_pkg</em> will be covered in the later tutorials about building ROS code.</p>
<h2 id="building-packages">4. Building Packages</h2>
<h3 id="using-catkin_make">4.1 Using catkin_make</h3>
<p><em><strong>catkin_make</strong></em> is a command line tool combing the calls to <code>cmake</code> and <code>make</code> in a standard CMake workflow.<br>
Usage:</p>
<pre><code># In a catkin workspace
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
</code></pre>
<pre><code># In a CMake project
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (optionally)
</code></pre>
<p>is equal to</p>
<pre><code># In a catkin workspace
$ catkin_make
$ catkin_make install  # (optionally)

# If /src has other name like /my_src
# In a catkin workspace
$ catkin_make --source my_src
$ catkin_make install --source my_src  # (optionally)
</code></pre>
<h3 id="building-your-package">4.2 Building Your Package</h3>
<p>Finished by above <code>catkin_make</code>, and in the log information we can see</p>
<ol>
<li>the default folder space</li>
</ol>
<pre><code>Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
</code></pre>
<ol start="2">
<li>the packages</li>
</ol>
<pre><code>-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</code></pre>
<h3 id="summary">4.3 Summary</h3>
<blockquote>
<p>Build the workplace, Setup environment, Build package, catkin_cmake</p>
</blockquote>
<p>After all those steps, in the catkin workspace there are three folders.</p>
<pre><code>$ ls ~/catkin_ws
	build
	devel
	src
</code></pre>

