from task import Task


class ListTask(Task):
    """Run a list of tasks sequentially"""

    def __init__(self, tasks, *args, **kwargs):
        super(ListTask, self).__init__(*args, **kwargs)

        self.tasks = tasks
        self.curr_index = 0

    def _on_task_run(self):
        if self.curr_index == len(self.tasks):
            self.finish()

        elif self.tasks[self.curr_index].finished:
            self.curr_index += 1

        else:
            self.tasks[self.curr_index].run()


class IndSimulTask(Task):
    """Run a list of tasks simulataneously, exits when all tasks finished"""

    def __init__(self, tasks, *args, **kwargs):
        super(IndSimulTask, self).__init__(*args, **kwargs)

        self.tasks = tasks

    def _on_task_run(self):
        self.all_finished = True
        for task in self.tasks:
            task.run()
            if not task.finished:
                self.all_finished = False

        if self.all_finished:
            self.finish()


class DepSimulTask(Task):
    """Run a list of tasks simulataneously, exits when any task finished"""

    def __init__(self, tasks, *args, **kwargs):
        super(DepSimulTask, self).__init__(*args, **kwargs)

        self.tasks = tasks

    def _on_task_run(self):
        self.any_finished = False
        for task in self.tasks:
            task.run()
            if task.finished:
                self.any_finished = True

        if self.any_finished:
            for task in self.tasks:
                task.finish
            self.finish()


class LeaderFollowerTask(Task):

    def __init__(self, leader, follower):
        super(LeaderFollowerTask, self).__init__()
        self.leader = leader
        self.follower = follower

    def _on_task_run(self):
        if self.leader.finished:
            self.follower.finish()
            self.finish()

        self.leader.run()
        self.follower.run()
