import numpy as np


# People tracker based on:
# https://pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/
# https://medium.com/aiguys/a-centroid-based-object-tracking-implementation-455021c2c997
class PeopleTracker:
    def __init__(self, max_frames_disappeared=50, max_euclidean_distance=50):
        self.id_counter = 0
        self.people = {}
        self.person_frames_disappeared = {}
        self.max_frames_disappeared = max_frames_disappeared
        self.max_euclidean_distance = max_euclidean_distance

    def update(self, bboxes):
        # Check if bboxes is empty and update frame disappeared count and delete if above threshold
        if len(bboxes) == 0:
            for person_id in list(self.people.keys()):
                self.person_frames_disappeared[person_id] += 1
                if self.person_frames_disappeared[person_id] > self.max_frames_disappeared:
                    del self.person_frames_disappeared[person_id]
                    del self.people[person_id]
        # Case there are bboxes
        else:
            # Compute centroids for bboxes
            centroids = [
                [topleft_x + ((botright_x - topleft_x) // 2), topleft_y + ((botright_y - topleft_y) // 2)]
                for [topleft_x, topleft_y, botright_x, botright_y]
                in bboxes
            ]
            # If there exists no people, then add the new centroids for the new people
            if len(self.people) == 0:
                for centroid in centroids:
                    self.register_new_person(centroid)
            # Case there already exists people and it it has received bboxes.
            # It must either, assign the new centroids to existing person objects,
            # or create a new person object and assign the centroid to this, for each new centroid
            else:
                # Compute euclidean distances list in the form of:
                # [[person_id, centroid_new, distance], .....]
                euclidean_distances = [
                                       [person_id, centroid_new, np.linalg.norm((self.people[person_id][0] - centroid_new[0], self.people[person_id][1] - centroid_new[1]))]
                                       for person_id in self.people.keys()
                                       for centroid_new in centroids
                                       ]
                # Sorts the list by the distances
                euclidean_distances.sort(key=lambda elem: elem[2])

                # Used to check whether the persons centroid has already been updated. Where the 'centroids'
                # list is used to check whether the new centroid has already been assigned
                unchanged_people = list(self.people.keys())
                # Assign the new centroids to the closest people centroids or create new
                # people object and assign centroid to this.
                for [person_id, centroid_new, distance] in euclidean_distances:
                    if person_id in unchanged_people and centroid_new in centroids:
                        if distance < self.max_euclidean_distance:
                            self.people[person_id] = centroid_new
                            unchanged_people.remove(person_id)
                            centroids.remove(centroid_new)
                # If there are still new unassigned centroids, then new people objects
                # are created and assigned these centroids
                for centroid_new in centroids:
                    self.register_new_person(centroid_new)
                # If som people were not assigned new centroid points, then add up their
                # frame disappeared count
                for person_id in unchanged_people:
                    self.person_frames_disappeared[person_id] += 1
                    if self.person_frames_disappeared[person_id] > self.max_frames_disappeared:
                        del self.person_frames_disappeared[person_id]
                        del self.people[person_id]
        return self.people

    def register_new_person(self, centroid):
        self.people[self.id_counter] = centroid
        self.person_frames_disappeared[self.id_counter] = 0
        self.id_counter += 1
