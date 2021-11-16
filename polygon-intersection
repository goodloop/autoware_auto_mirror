### POLYGON INTERSECTION

**The Problem:** Given two convex polygons, compute their intersection

**The Solution:** You could use a Polygon Clipping algorithm to find the intersection between two polygons. However these tend to be complicated algorithms when all of the edge cases are taken into account.

There are 2 most used polygon clipping algorithms:
  * Sutherland–Hodgman Algorithm
  * Weiler–Atherton Algorithm

**1. Sutherland–Hodgman Polygon Clipping Algorithm**

The algorithm begins with an input list of all vertices in the subject polygon. Next, one side of the clip polygon is extended infinitely in both directions, and the path of the subject polygon is traversed. Vertices from the input list are inserted into an output list if they lie on the visible side of the extended clip polygon line, and new vertices are added to the output list where the subject polygon path crosses the extended clip polygon line.

This process is repeated iteratively for each clip polygon side, using the output list from one stage as the input list for the next. Once all sides of the clip polygon have been processed, the final generated list of vertices defines a new single polygon that is entirely visible. 


![img_2.png](img_2.png)

**2.Weiler-Atherton Polygon Clipping Algorithm** 

The Weiler–Atherton is a polygon-clipping algorithm. It is used in areas like computer graphics and games development where clipping of polygons is needed. It allows clipping of a subject or candidate polygon by an arbitrarily shaped clipping polygon/area/region. It is generally applicable only in 2D.

Steps:

1. First make a list of all intersection points namely i1, i2, i3, ...
2. Classify those intersection points as entering or exiting.
3. Now, make two lists, one for the clipping polygon, and the other for the clipped polygon.
4. Fill both the lists up in such a way that the intersection points lie between the correct vertices of each of the polygon. That is the clipping polygon list is filled up with all the vertices of the clipping polygon along with the intersecting points lying between the corresponding vertices.
5. Now, start at the 'to be clipped' polygon's list.
6. Choose the first intersection point which has been labelled as an entering point. Follow the points in the list (looping back to the top of the list, in case the list ends) and keep on pushing them into a vector or something similar of the sorts. Keep on following the list until an exiting intersection point is found.
7. Now switch the list to the 'polygon that is clipping' list, and find the exiting the intersection that was previously encountered. Now keep on following the points in this list (similar to how we followed the previous list) until the entering intersection point is found (the one that was found in the previous 'to be clipped' polygon's list).
8. This vector now formed by pushing all the encountered points in the two lists, is now the clipped polygon (one of the many clipped polygons if any of the clipping polygons is concave).
9. Repeat this clipping procedure (i.e. from step 5) until all the entering intersection points have been visited once.


![img_3.png](img_3.png)

**Boost.Geometry library uses an adapted version of the Weiler-Atherton algorithm in the clipping algorithms mentioned above.**

**SHA vs WAA** 

The Weiler–Atherton algorithm is more complex and computationally more expensive then Sutherland–Hodgman algorithm. But there is a very important point here. **SHA works for only convex polygons.** Note that if the subject polygon was concave at vertices outside the clipping polygon, the new polygon may have coincident (i.e., overlapping) edges – this is acceptable for rendering, but not for other applications such as computing shadows. On the other hand Weiler Atherton Polygon Clipping Algorithm is an algorithm made to allow clipping of even concave algorithms to be possible. Unlike Sutherland – Hodgman polygon clipping algorithm, this algorithm is able to clip concave polygons without leaving any residue behind.

