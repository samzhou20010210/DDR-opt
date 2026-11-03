# A rescue order: bring person `person_id` to `extraction_zone`.
std_msgs/Header header
string task_id                  # unique task identifier
string person_id                # must match Person.id
pool_rescue/ExtractionZone extraction_zone
